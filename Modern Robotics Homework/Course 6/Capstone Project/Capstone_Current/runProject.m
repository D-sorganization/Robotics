% runProject.m - Complete Mobile Manipulation Project Runner
% This script runs the entire project and generates all required outputs

clear; clc; close all;

fprintf('================================================\n');
fprintf('  Mobile Manipulation Capstone Project\n');
fprintf('  Northwestern University - Modern Robotics\n');
fprintf('================================================\n\n');

%% Add Modern Robotics library to path
% Make sure the Modern Robotics library is in your path
% addpath('path/to/ModernRobotics/packages/MATLAB/mr');

%% Robot parameters (KUKA youBot)
fprintf('Initializing robot parameters...\n');

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

%% Cube configurations
% Initial cube position
Tsc_initial = RpToTrans(eye(3), [1; 0; 0.025]);

% Final cube position (rotated -90 degrees about z)
Tsc_final = RpToTrans(RotZ(-pi/2), [0; -1; 0.025]);

%% Grasp configuration
% Approach angle
a = pi/5;  % 36 degrees

% End-effector configuration relative to cube
Tce_standoff = [-sin(a), 0, -cos(a), 0;
                 0,      1,  0,       0;
                 cos(a), 0, -sin(a), 0.1;
                 0,      0,  0,       1];

Tce_grasp = [-sin(a), 0, -cos(a), 0;
              0,      1,  0,       0;
              cos(a), 0, -sin(a), 0;
              0,      0,  0,       1];

%% Reference trajectory initial configuration
Tse_initial = [0, 0, 1, 0.5;
               0, 1, 0, 0;
              -1, 0, 0, 0.5;
               0, 0, 0, 1];

fprintf('Initialization completed\n');

%% Generate reference trajectory
fprintf('\nGenerating reference trajectory...\n');
dt = 0.01;  % Time step (10 ms)
k = 1;      % Trajectory points per timestep

[Traj, gripper_state] = TrajectoryGenerator(Tse_initial, Tsc_initial, ...
                                            Tsc_final, Tce_grasp, ...
                                            Tce_standoff, k);

% Save reference trajectory
writematrix(Traj, 'Traj.csv');
fprintf('Trajectory generated and saved to Traj.csv\n');
fprintf('Total trajectory points: %d\n', size(Traj, 1));
fprintf('Total trajectory time: %.1f seconds\n', size(Traj, 1) * dt);

%% Initial robot configuration (with error from reference)
config_initial = [0;         % phi
                  0;         % x  
                  0;         % y
                  0;         % J1
                  0;         % J2
                  0.2;       % J3
                  -1.67;     % J4
                  0;         % J5
                  0;         % W1
                  0;         % W2
                  0;         % W3
                  0];        % W4

%% Controller gains
Kp = 1.5* eye(6);  % Proportional gain
Ki = 0.01 * eye(6);    % Integral gain (set to 0 for P control only)

%% Simulation parameters
max_speed = 12.3;  % Maximum speed for joints and wheels (rad/s)

%% Convert trajectory to SE(3) format
fprintf('\nRunning feedback control simulation...\n');

N_traj = size(Traj, 1);
Td = zeros(4, 4, N_traj);

for i = 1:N_traj
    % Extract rotation matrix (stored row-wise in Traj)
    R = [Traj(i,1:3); Traj(i,4:6); Traj(i,7:9)];
    p = Traj(i,10:12)';
    Td(:,:,i) = RpToTrans(R, p);
end

%% Run feedback control simulation
config = config_initial;
integral_error = zeros(6, 1);
xerr_prev = zeros(6, 1);  % Error from previous timestep for integral term
Animation = zeros(N_traj, 13);
Xerr = zeros(6, N_traj-1);

% Store initial configuration
Animation(1,:) = [config', gripper_state(1)];

% Main control loop
for i = 1:N_traj-1
    % Update integral error with error from previous step
    integral_error = integral_error + xerr_prev * dt;

    % Current and next reference configurations
    Xd = Td(:,:,i);
    Xd_next = Td(:,:,i+1);

    % Compute current end-effector configuration
    [Tse, Je] = youBotKinematics(config);

    % Feedback control
    [V, xerr_prev] = FeedbackControl(Tse, Xd, Xd_next, Kp, Ki, dt, integral_error);
    Xerr(:,i) = xerr_prev;
    
    % Check and apply joint limits
    violated_joints = checkJointLimits(config(4:8));
    Je = applyJointLimits(Je, violated_joints);
    
    % Calculate joint speeds
    speeds = pinv(Je, 1e-3) * V;
    
    % Apply speed limits
    speeds(speeds > max_speed) = max_speed;
    speeds(speeds < -max_speed) = -max_speed;
    
    % Update configuration
    config = NextState(config, speeds, dt, max_speed);
    
    % Store configuration
    Animation(i+1,:) = [config', gripper_state(i+1)];
    
    % Progress indicator
    if mod(i, 500) == 0
        fprintf('  Progress: %.1f%%\n', 100*i/(N_traj-1));
    end
end

fprintf('Feedback control completed\n');

%% Save animation file
writematrix(Animation, 'Animation.csv');
fprintf('\nAnimation file saved to Animation.csv\n');

%% Plot error
figure('Position', [100, 100, 800, 600]);
time = (0:N_traj-2) * dt;
plot(time, Xerr', 'LineWidth', 1.5);
title('Error Twist vs Time', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 12);
ylabel('Error Twist', 'FontSize', 12);
legend('\omega_x', '\omega_y', '\omega_z', 'v_x', 'v_y', 'v_z', ...
       'Location', 'best');
grid on;

% Save plot
saveas(gcf, 'error_plot.pdf');
saveas(gcf, 'error_plot.png');

% Save error data
save('Xerr.mat', 'Xerr');
fprintf('Error plot and data saved\n');

%% Calculate and display statistics
error_norms = sqrt(sum(Xerr.^2, 1));
final_error = error_norms(end);
max_error = max(error_norms);

% Find convergence time
threshold = 0.01;
converged_idx = find(error_norms < threshold, 1);
if ~isempty(converged_idx)
    convergence_time = converged_idx * dt;
else
    convergence_time = inf;
end

%% Create summary report
fid = fopen('simulation_summary.txt', 'w');
fprintf(fid, 'Mobile Manipulation Capstone - Simulation Summary\n');
fprintf(fid, '================================================\n\n');
fprintf(fid, 'Controller Parameters:\n');
fprintf(fid, '  Proportional Gain (Kp): %.1f\n', Kp(1,1));
fprintf(fid, '  Integral Gain (Ki): %.1f\n', Ki(1,1));
fprintf(fid, '\n');
fprintf(fid, 'Simulation Parameters:\n');
fprintf(fid, '  Time Step: %.3f s\n', dt);
fprintf(fid, '  Max Speed: %.1f rad/s\n', max_speed);
fprintf(fid, '  Total Time: %.1f s\n', (N_traj-1)*dt);
fprintf(fid, '\n');
fprintf(fid, 'Performance Metrics:\n');
fprintf(fid, '  Final Error Norm: %.6f\n', final_error);
fprintf(fid, '  Maximum Error Norm: %.6f\n', max_error);
if isfinite(convergence_time)
    fprintf(fid, '  Convergence Time: %.2f s\n', convergence_time);
else
    fprintf(fid, '  Convergence Time: Did not converge\n');
end
fprintf(fid, '\n');
fprintf(fid, 'Output Files:\n');
fprintf(fid, '  - Traj.csv: Reference trajectory\n');
fprintf(fid, '  - Animation.csv: Robot motion for CoppeliaSim\n');
fprintf(fid, '  - Xerr.mat: Error data\n');
fprintf(fid, '  - error_plot.pdf/png: Error visualization\n');
fclose(fid);

%% Display summary
fprintf('\n================================================\n');
fprintf('  Simulation Complete!\n');
fprintf('================================================\n\n');
fprintf('Performance Summary:\n');
fprintf('  Final Error Norm: %.6f\n', final_error);
fprintf('  Maximum Error Norm: %.6f\n', max_error);
if isfinite(convergence_time)
    fprintf('  Convergence Time: %.2f s\n', convergence_time);
else
    fprintf('  Did not converge to threshold %.3f\n', threshold);
end
fprintf('\nFiles Generated:\n');
fprintf('  1. Traj.csv - Reference trajectory (%d configurations)\n', N_traj);
fprintf('  2. Animation.csv - Robot animation for CoppeliaSim\n');
fprintf('  3. Xerr.mat - Error data\n');
fprintf('  4. error_plot.pdf/png - Error visualization\n');
fprintf('  5. simulation_summary.txt - Summary report\n');
fprintf('\nTo visualize in CoppeliaSim:\n');
fprintf('  1. Open Scene 6: CSV Mobile Manipulation youBot\n');
fprintf('  2. Load Animation.csv\n');
fprintf('  3. Click "Play File"\n');
fprintf('\n');