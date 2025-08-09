function [traj, gripper_state] = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k)
% TrajectoryGenerator - Generates reference trajectory for end-effector
% This version uses Modern Robotics library functions
% Inputs:
%   Tse_initial - Initial end-effector configuration (4x4 SE(3))
%   Tsc_initial - Initial cube configuration (4x4 SE(3))
%   Tsc_final - Final cube configuration (4x4 SE(3))
%   Tce_grasp - End-effector config relative to cube when grasping (4x4)
%   Tce_standoff - Standoff configuration above cube (4x4)
%   k - Number of trajectory points per 0.01 seconds
%
% Outputs:
%   traj - Nx13 matrix where each row is:
%          [r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,gripper]
%   gripper_state - Nx1 vector of gripper states (0=open, 1=closed)

% Note: This code assumes the Modern Robotics library is in the MATLAB path

% Key configurations
Tse_standoff_initial = Tsc_initial * Tce_standoff;
Tse_grasp_initial = Tsc_initial * Tce_grasp;
Tse_standoff_final = Tsc_final * Tce_standoff;
Tse_grasp_final = Tsc_final * Tce_grasp;

% Calculate distances for time allocation
d1 = norm(Tse_initial(1:3,4) - Tse_standoff_initial(1:3,4));
d2 = norm(Tse_standoff_initial(1:3,4) - Tse_grasp_initial(1:3,4));
d5 = norm(Tse_standoff_initial(1:3,4) - Tse_standoff_final(1:3,4));
d6 = norm(Tse_standoff_final(1:3,4) - Tse_grasp_final(1:3,4));

% Fixed times for gripper operations
t_grasp = 0.63;  % Time to close gripper
t_release = 0.63;  % Time to open gripper

% Total time and time allocation based on distances
T_total = 15;  % Total trajectory time (seconds)
T_motion = T_total - t_grasp - t_release;

% Allocate time proportionally to distances
d_total = d1 + 2*d2 + d5 + 2*d6;
t1 = max(1, round(d1 * T_motion / d_total * 100) / 100);
t2 = max(0.5, round(d2 * T_motion / d_total * 100) / 100);
t4 = t2;  % Same time for lifting as lowering
t5 = max(1, round(d5 * T_motion / d_total * 100) / 100);
t6 = max(0.5, round(d6 * T_motion / d_total * 100) / 100);
t8 = t6;  % Same time for lifting as lowering

% Adjust if needed to ensure total time is correct
time_sum = t1 + t2 + t_grasp + t4 + t5 + t6 + t_release + t8;
if abs(time_sum - T_total) > 0.1
    scale = (T_total - t_grasp - t_release) / (t1 + t2 + t4 + t5 + t6 + t8);
    t1 = t1 * scale; t2 = t2 * scale; t4 = t4 * scale;
    t5 = t5 * scale; t6 = t6 * scale; t8 = t8 * scale;
end

% Time step
dt = 0.01 / k;

% Initialize trajectory storage
traj = [];
gripper_state = [];

% Configuration sequence for 8 segments
T_configs = {Tse_initial, Tse_standoff_initial, Tse_grasp_initial, ...
             Tse_grasp_initial, Tse_standoff_initial, Tse_standoff_final, ...
             Tse_grasp_final, Tse_grasp_final, Tse_standoff_final};
             
segment_times = [t1, t2, t_grasp, t4, t5, t6, t_release, t8];
gripper_states = [0, 0, 1, 1, 1, 1, 0, 0];

% Generate trajectory for each segment
for seg = 1:8
    T_start = T_configs{seg};
    T_end = T_configs{seg+1};
    T_seg = segment_times(seg);
    N_seg = max(2, round(T_seg / dt));
    
    % Use appropriate trajectory generation based on segment type
    if seg == 3 || seg == 7  % Gripper operation segments (stay in place)
        % Stay at the same configuration
        traj_cell = cell(N_seg, 1);
        for i = 1:N_seg
            traj_cell{i} = T_start;
        end
    else
        % Use ScrewTrajectory for motion segments
        % Note: Using method 5 (quintic time scaling) for smooth acceleration
        traj_cell = ScrewTrajectory(T_start, T_end, T_seg, N_seg, 5);
    end
    
    % Convert cell array to matrix format
    traj_seg = cell2mat_trajectory(traj_cell, gripper_states(seg));
    
    % Append to full trajectory
    traj = [traj; traj_seg];
    gripper_state = [gripper_state; gripper_states(seg) * ones(N_seg, 1)];
end

end

function traj_matrix = cell2mat_trajectory(traj_cell, gripper)
% Convert cell array of SE(3) matrices to N x 13 matrix format
    N = length(traj_cell);
    traj_matrix = zeros(N, 13);
    
    for i = 1:N
        T = traj_cell{i};
        R = T(1:3, 1:3);
        p = T(1:3, 4);
        
        % Store as row vector [r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,gripper]
        % Note: R is stored row-wise
        traj_matrix(i,:) = [R(1,:), R(2,:), R(3,:), p', gripper];
    end
end