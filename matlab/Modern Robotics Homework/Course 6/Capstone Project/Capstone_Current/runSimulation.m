function runSimulation(task_type)
% runSimulation - Main program to run the mobile manipulation simulation
% Input:
%   task_type - 'best', 'overshoot', or 'newTask'

if nargin < 1
    task_type = 'best';
end

fprintf('Running simulation: %s\n', task_type);

%% Robot parameters (fixed for youBot)
% Mobile base parameters
l = 0.235;    % half-length of robot (m)
w = 0.15;     % half-width of robot (m) 
r = 0.0475;   % wheel radius (m)

% Transformation from base to arm base frame {0}
Tb0 = RpToTrans(eye(3), [0.1662; 0; 0.0026]);

% Home configuration of end-effector in arm frame {0}
M0e = RpToTrans(eye(3), [0.033; 0; 0.6546]);

% Screw axes in end-effector frame at home configuration
Blist = [0,  0,  1,  0,      0.033, 0;
         0, -1,  0, -0.5076, 0,     0;
         0, -1,  0, -0.3526, 0,     0;
         0, -1,  0, -0.2176, 0,     0;
         0,  0,  1,  0,      0,     0]';

%% Define cube configurations
if strcmp(task_type, 'newTask')
    % Custom task with different cube positions
    Tsc_initial = RpToTrans(eye(3), [0.5; 0.5; 0.025]);
    Tsc_final = RpToTrans(RotZ(-pi/2), [-0.5; -0.5; 0.025]);
else
    % Default cube configurations
    Tsc_initial = RpToTrans(eye(3), [1; 0; 0.025]);
    Tsc_final = RpToTrans(RotZ(-pi/2), [0; -1; 0.025]);
end

%% Define grasp configurations
% Approach frame tilted about y by 36 degrees to match reference project
a = pi/5;  % [rad] tilt angle from functional project specification
% Rotation matrix for end-effector approach relative to cube frame
R_y = [-sin(a), 0,  cos(a);
       0,       1,  0;
      -cos(a), 0, -sin(a)];

% Grasp and standoff expressed relative to the cube frame
Tce_grasp = RpToTrans(R_y, [0; 0; 0]);
Tce_standoff = RpToTrans(R_y, [0; 0; 0.25]);

% Precompute world-frame configurations
Tse_standoff_initial = Tsc_initial * Tce_standoff;
Tse_grasp_initial    = Tsc_initial * Tce_grasp;
Tse_standoff_final   = Tsc_final * Tce_standoff;
Tse_grasp_final      = Tsc_final * Tce_grasp;

%% Define reference trajectory initial configuration
Tse_initial = [0, 0, 1, 0.5;
               0, 1, 0, 0;
              -1, 0, 0, 0.5;
               0, 0, 0, 1];

%% Define robot initial configuration with error
% Create initial configuration with at least 30 deg rotation error and 0.2m position error
if strcmp(task_type, 'best') || strcmp(task_type, 'newTask')
    config_initial = [0;         % phi
                      0;         % x
                      0;         % y
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
    config_initial = [pi/6;      % phi - 30 degrees
                      0.2;       % x - offset
                      0.1;       % y - offset
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
        Kp = 1.5 * eye(6);
        Ki = 0.2 * eye(6);
        
    case 'overshoot'
        % Less-well-tuned controller with overshoot
        Kp = 20 * eye(6);
        Ki = 0.5 * eye(6);
        
    case 'newTask'
        % Custom gains for new task
        Kp = 2 * eye(6);
        Ki = 0.1 * eye(6);
end

%% Simulation parameters
dt = 0.01;              % Timestep (10 ms)
max_speed = 12.3;       % Maximum joint/wheel speed (rad/s)
k = 1;                  % Trajectory points per 0.01s

%% Generate reference trajectory
fprintf('Generating reference trajectory...\n');
[traj_ref, gripper_ref] = TrajectoryGenerator(Tse_initial, Tsc_initial, ...
                                               Tsc_final, Tce_grasp, ...
                                               Tce_standoff, k);

% Save reference trajectory
traj_filename = [task_type, '_Traj.csv'];
dlmwrite(traj_filename, traj_ref, 'delimiter', ',', 'precision', '%.6f');
fprintf('Reference trajectory saved: %s\n', traj_filename);

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
Xerr_prev = zeros(6, 1);  % Error from previous timestep for integral term
config_log = zeros(N_traj, 13);
Xerr_log = zeros(6, N_traj-1);

% Store initial configuration
config_log(1,:) = [config', gripper_ref(1)];

%% Main simulation loop
fprintf('Running feedback control simulation...\n');
for i = 1:N_traj-1
    % Update integral error with previous error
    integral_error = integral_error + Xerr_prev * dt;

    % Current and next reference configurations
    Xd = traj_SE3(:,:,i);
    Xd_next = traj_SE3(:,:,i+1);
    gripper_state = gripper_ref(i);

    % Compute current end-effector configuration
    [X, Je] = youBotKinematics(config);

    % Feedback control (use integral_error accumulated up to previous step)
    [V, Xerr_prev] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, integral_error);

    % Check and apply conservative joint limits
    violated_joints = checkJointLimits(config(4:8));
    Je = applyJointLimits(Je, violated_joints);
    
    % Calculate joint speeds using pseudoinverse
    speeds = pinv(Je, 1e-3) * V;
    
    % Apply speed limits
    speeds(speeds > max_speed) = max_speed;
    speeds(speeds < -max_speed) = -max_speed;
    
    % Update configuration
    config = NextState(config, speeds, dt, max_speed);
    
    % Log data
    config_log(i+1,:) = [config', gripper_state];
    Xerr_log(:,i) = Xerr_prev;
    
    % Progress indicator
    if mod(i, 500) == 0
        fprintf('  Progress: %.1f%%%%\n', 100*i/(N_traj-1));
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

% Write animation CSV file
csv_filename = [results_dir, '/Animation.csv'];
dlmwrite(csv_filename, config_log, 'delimiter', ',', 'precision', '%.6f');
fprintf('Animation CSV saved: %s\n', csv_filename);

% Save error data
error_filename = [results_dir, '/Xerr.mat'];
save(error_filename, 'Xerr_log');

% Plot and save error
figure('Position', [100, 100, 800, 600]);
time = (0:N_traj-2) * dt;
plot(time, Xerr_log', 'LineWidth', 1.5);
title('Error Twist vs Time', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 12);
ylabel('Error Twist', 'FontSize', 12);
legend('\omega_x', '\omega_y', '\omega_z', 'v_x', 'v_y', 'v_z', 'Location', 'best');
grid on;

plot_filename = [results_dir, '/error_plot.pdf'];
saveas(gcf, plot_filename);
fprintf('Error plot saved: %s\n', plot_filename);

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
final_error_norm = norm(Xerr_log(:,end));
max_error_norm = max(sqrt(sum(Xerr_log.^2, 1)));
fprintf(fid, '\nFinal Error Norm: %.6f\n', final_error_norm);
fprintf(fid, 'Maximum Error Norm: %.6f\n', max_error_norm);

% Calculate convergence time (when error < threshold)
threshold = 0.01;
error_norms = sqrt(sum(Xerr_log.^2, 1));
converged_idx = find(error_norms < threshold, 1);
if ~isempty(converged_idx)
    convergence_time = converged_idx * dt;
    fprintf(fid, 'Convergence Time (error < %.3f): %.2f s\n', threshold, convergence_time);
else
    fprintf(fid, 'Did not converge to threshold %.3f\n', threshold);
end

fclose(fid);

fprintf('\n=== Simulation Complete! ===\n');
fprintf('Results saved in: %s/\n', results_dir);
fprintf('Final error norm: %.6f\n', final_error_norm);
fprintf('Files generated:\n');
fprintf('  - %s (animation file for CoppeliaSim)\n', csv_filename);
fprintf('  - %s (error data)\n', error_filename);
fprintf('  - %s (error plot)\n', plot_filename);
fprintf('  - %s (trajectory reference)\n', traj_filename);
fprintf('\n');

end