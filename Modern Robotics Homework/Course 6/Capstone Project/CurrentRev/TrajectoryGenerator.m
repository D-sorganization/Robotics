function [traj, gripper_state] = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k)
% TrajectoryGenerator - Generates reference trajectory for end-effector
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

% Trajectory segment durations (seconds)
t1 = 5;  % Move to initial standoff
t2 = 2;  % Move down to grasp
t3 = 0.63;  % Close gripper (must be >= 0.625s)
t4 = 2;  % Move back to standoff
t5 = 5;  % Move to final standoff
t6 = 2;  % Move down to release
t7 = 0.63;  % Open gripper
t8 = 2;  % Move back to final standoff

% Number of points per segment
N = @(t) ceil(t * k / 0.01);

% Key configurations
Tse_standoff_initial = Tsc_initial * Tce_standoff;
Tse_grasp_initial = Tsc_initial * Tce_grasp;
Tse_standoff_final = Tsc_final * Tce_standoff;
Tse_grasp_final = Tsc_final * Tce_grasp;

% Initialize trajectory storage
traj = [];
gripper_state = [];

% Segment 1: Initial to standoff above initial cube
[traj_seg, grip_seg] = generateSegment(Tse_initial, Tse_standoff_initial, N(t1), 0);
traj = [traj; traj_seg];
gripper_state = [gripper_state; grip_seg];

% Segment 2: Lower to grasp position
[traj_seg, grip_seg] = generateSegment(Tse_standoff_initial, Tse_grasp_initial, N(t2), 0);
traj = [traj; traj_seg];
gripper_state = [gripper_state; grip_seg];

% Segment 3: Close gripper (stay in place)
[traj_seg, grip_seg] = generateSegment(Tse_grasp_initial, Tse_grasp_initial, N(t3), 1);
traj = [traj; traj_seg];
gripper_state = [gripper_state; grip_seg];

% Segment 4: Lift back to standoff
[traj_seg, grip_seg] = generateSegment(Tse_grasp_initial, Tse_standoff_initial, N(t4), 1);
traj = [traj; traj_seg];
gripper_state = [gripper_state; grip_seg];

% Segment 5: Move to standoff above final position
[traj_seg, grip_seg] = generateSegment(Tse_standoff_initial, Tse_standoff_final, N(t5), 1);
traj = [traj; traj_seg];
gripper_state = [gripper_state; grip_seg];

% Segment 6: Lower to release position
[traj_seg, grip_seg] = generateSegment(Tse_standoff_final, Tse_grasp_final, N(t6), 1);
traj = [traj; traj_seg];
gripper_state = [gripper_state; grip_seg];

% Segment 7: Open gripper (stay in place)
[traj_seg, grip_seg] = generateSegment(Tse_grasp_final, Tse_grasp_final, N(t7), 0);
traj = [traj; traj_seg];
gripper_state = [gripper_state; grip_seg];

% Segment 8: Lift back to final standoff
[traj_seg, grip_seg] = generateSegment(Tse_grasp_final, Tse_standoff_final, N(t8), 0);
traj = [traj; traj_seg];
gripper_state = [gripper_state; grip_seg];

end

function [traj_matrix, gripper_vec] = generateSegment(T_start, T_end, N, gripper)
% Generate trajectory segment using screw motion
% Uses quintic time scaling for smooth motion

    % Initialize output
    traj_matrix = zeros(N, 13);
    gripper_vec = gripper * ones(N, 1);
    
    % Generate trajectory using matrix exponential
    for i = 1:N
        s = (i-1)/(N-1);  % Time scaling parameter [0,1]
        
        % Quintic time scaling for smooth acceleration
        s_scaled = 10*s^3 - 15*s^4 + 6*s^5;
        
        % Compute intermediate configuration
        T = T_start * MatrixExp6(MatrixLog6(TransInv(T_start) * T_end) * s_scaled);
        
        % Extract rotation and position
        R = T(1:3, 1:3);
        p = T(1:3, 4);
        
        % Store as row vector [r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,gripper]
        traj_matrix(i,:) = [R(1,:), R(2,:), R(3,:), p', gripper];
    end
end

% Note: This code assumes the Modern Robotics library is in the MATLAB path
% All helper functions (MatrixExp6, MatrixLog6, TransInv) are from that library