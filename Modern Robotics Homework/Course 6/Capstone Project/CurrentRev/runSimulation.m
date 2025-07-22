function runSimulation(task_type)
% runSimulation - Main program to run the mobile manipulation simulation
% Input:
%   task_type - 'best', 'overshoot', or 'newTask'

if nargin < 1
    task_type = 'best';
end

fprintf('Running simulation: %s\n', task_type);

%% Define cube configurations
if strcmp(task_type, 'newTask')
    % Custom task with different cube positions
    Tsc_initial = [1, 0, 0, 0.5;    % Different initial position
                   0, 1, 0, 0.5;
                   0, 0, 1, 0.025;
                   0, 0, 0, 1];
    
    Tsc_final = [0, 1, 0, -0.5;     % Different final position
                -1, 0, 0, -0.5;
                 0, 0, 1, 0.025;
                 0, 0, 0, 1];
else
    % Default cube configurations
    Tsc_initial = [1, 0, 0, 1;
                   0, 1, 0, 0;
                   0, 0, 1, 0.025;
                   0, 0, 0, 1];
    
    Tsc_final = [0, 1, 0, 0;
                -1, 0, 0, -1;
                 0, 0, 1, 0.025;
                 0, 0, 0, 1];
end

%% Define grasp configurations
theta = pi/2;
Tce_grasp = [cos(theta), 0, sin(theta), 0;
             0,          1, 0,          0;
            -sin(theta), 0, cos(theta), 0;
             0,          0, 0,          1];

Tce_standoff = [cos(theta), 0, sin(theta), 0;
                0,          1, 0,          0;
               -sin(theta), 0, cos(theta), 0.1;
                0,          0, 0,          1];

%% Define reference trajectory initial configuration
Tse_initial = [0, 0, 1, 0.5;
               0, 1, 0, 0;
              -1, 0, 0, 0.5;
               0, 0, 0, 1];

%% Define robot initial configuration with error
% Create initial configuration with at least 30 deg rotation error and 0.2m position error
if strcmp(task_type, 'best') || strcmp(task_type, 'newTask')
    config_initial = [pi/6;      % phi - 30 degrees
                      0.3;       % x - 0.3m offset
                      0.2;       % y - 0.2m offset
                      0;         % J1
                      0;         % J2
                      0.2;       % J3
                      -1.6;      % J4
                      0;         % J5
                      0;         % W1
                      0;         % W2
                      0;         % W3
                      0];        % W4
else % overshoot
    config_initial = [pi/4;      % phi - 45 degrees
                      0.4;       % x - larger offset
                      0.3;       % y - larger offset
                      0;         % J1
                      -0.2;      % J2
                      0.3;       % J3
                      -1.5;      % J4
                      0;         % J5
                      0;         % W1
                      0;         % W2
                      0;         % W3
                      0];        % W4
end

%% Set control gains based on task type
switch task_type
    case 'best'
        % Well-tuned PI controller
        Kp = 2 * eye(6);
        Ki = 0.1 * eye(6);
        
    case 'overshoot'
        % Less-well-tuned controller with overshoot
        Kp = 5 * eye(6);
        Ki = 0.5 * eye(6);
        
    case 'newTask'
        % Custom gains for new task
        Kp = 2.5 * eye(6);
        Ki = 0.15 * eye(6);
end

%% Simulation parameters
dt = 0.01;              % Timestep (10 ms)
max_speed = 10;         % Maximum joint/wheel speed (rad/s)
k = 1;                  % Trajectory points per 0.01s

%% Generate reference trajectory
fprintf('Generating reference trajectory...\n');
[traj_ref, gripper_ref] = TrajectoryGenerator(Tse_initial, Tsc_initial, ...
                                               Tsc_final, Tce_grasp, ...
                                               Tce_standoff, k);

% Convert trajectory format to 4x4 matrices
N_traj = size(traj_ref, 1);
traj_SE3 = zeros(4, 4, N_traj);
for i = 1:N_traj
    R = [traj_ref(i,1:3); traj_ref(i,4:6); traj_ref(i,7:9)];
    p = traj_ref(i,10:12)';
    traj_SE3(:,:,i) = [R, p; 0 0 0 1];
end

%% Initialize simulation
config = config_initial;
integral_error = zeros(6, 1);
config_log = zeros(N_traj-1, 13);
Xerr_log = zeros(N_traj-1, 6);

%% Main simulation loop
fprintf('Running simulation...\n');
for i = 1:N_traj-1
    % Current and next reference configurations
    Xd = traj_SE3(:,:,i);
    Xd_next = traj_SE3(:,:,i+1);
    gripper_state = gripper_ref(i);
    
    % Compute current end-effector configuration
    [X, Je] = youBotKinematics(config);
    
    % Feedback control
    [V, Xerr] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, integral_error);
    
    % Update integral error
    integral_error = integral_error + Xerr * dt;
    
    % Check joint limits (optional)
    arm_angles = config(4:8);
    violated_joints = checkJointLimits(arm_angles);
    if ~isempty(violated_joints)
        Je = applyJointLimits(Je, violated_joints);
    end
    
    % Calculate joint speeds using pseudoinverse
    % Use tolerance to handle near-singularities
    speeds = pinv(Je, 1e-3) * V;
    
    % Apply additional speed scaling if speeds are too large
    max_speed_commanded = max(abs(speeds));
    if max_speed_commanded > max_speed
        speeds = speeds * (max_speed / max_speed_commanded);
    end
    
    % Update configuration
    config = NextState(config, speeds, dt, max_speed);
    
    % Log data
    config_log(i,:) = [config', gripper_state];
    Xerr_log(i,:) = Xerr';
    
    % Progress indicator
    if mod(i, 100) == 0
        fprintf('  Progress: %.1f%%\n', 100*i/(N_traj-1));
    end
end

%% Save results
fprintf('Saving results...\n');

% Create results directory if it doesn't exist
results_dir = ['results/', task_type];
if ~exist('results', 'dir')
    mkdir('results');
end
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

% Write CSV file
csv_filename = [results_dir, '/trajectory.csv'];
writeCSV(csv_filename, config_log(:,1:12), config_log(:,13));

% Plot and save error
plot_filename = [results_dir, '/error_plot.pdf'];
plotError(Xerr_log, plot_filename);

% Save error data
error_filename = [results_dir, '/error_data.mat'];
save(error_filename, 'Xerr_log');

% Write README
readme_filename = [results_dir, '/README.txt'];
fid = fopen(readme_filename, 'w');
fprintf(fid, 'Simulation Results: %s\n\n', task_type);
fprintf(fid, 'Controller Type: Feedforward + PI Control\n');
fprintf(fid, 'Proportional Gain (Kp): %.2f * I\n', Kp(1,1));
fprintf(fid, 'Integral Gain (Ki): %.2f * I\n', Ki(1,1));
fprintf(fid, 'Maximum Speed Limit: %.1f rad/s\n', max_speed);
fprintf(fid, 'Timestep: %.3f s\n', dt);
fprintf(fid, '\n');

if strcmp(task_type, 'newTask')
    fprintf(fid, 'Initial Cube Position: [%.2f, %.2f, %.3f]\n', ...
            Tsc_initial(1,4), Tsc_initial(2,4), Tsc_initial(3,4));
    fprintf(fid, 'Final Cube Position: [%.2f, %.2f, %.3f]\n', ...
            Tsc_final(1,4), Tsc_final(2,4), Tsc_final(3,4));
end

% Calculate final error
final_error = norm(Xerr_log(end,:));
fprintf(fid, '\nFinal Error Norm: %.6f\n', final_error);
fprintf(fid, 'Maximum Error Norm: %.6f\n', max(sqrt(sum(Xerr_log.^2, 2))));

fclose(fid);

fprintf('\nSimulation complete!\n');
fprintf('Results saved in: %s/\n', results_dir);
fprintf('Final error norm: %.6f\n', final_error);

end