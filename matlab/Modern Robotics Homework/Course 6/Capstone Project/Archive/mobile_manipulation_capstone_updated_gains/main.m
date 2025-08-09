clear; clc;

Tse_initial = [1,0,0,0.5; 0,1,0,0; 0,0,1,0.5; 0,0,0,1];
Tsc_initial = [1,0,0,1; 0,1,0,0; 0,0,1,0.025; 0,0,0,1];
Tsc_goal = [0,1,0,0; -1,0,0,-1; 0,0,1,0.025; 0,0,0,1];
Tce_grasp = [0,0,1,0; 0,1,0,0; -1,0,0,0; 0,0,0,1];
Tce_standoff = [0,0,1,0; 0,1,0,0; -1,0,0,0.2; 0,0,0,1];

[traj, gripper_state] = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff);
Kp = 7 * eye(6);
Ki = 1.0 * eye(6);
config = [0; 0; 0; 0; 0; 0.2; -1.6; 0; 0; 0; 0; 0];
dt = 0.01;
max_speed = 12.3;
config_list = [];
Xerr_log = [];

for i = 1:size(traj,1)
    T = [reshape(traj(i,1:12), 3, 4); 0 0 0 1];
    gripper = gripper_state(i);
    [Tb0, Fraw] = youBotKinematics();
    arm_joint_angles = config(4:8);
    T0e = manualFK(arm_joint_angles);
    X = configToSE3(config(1:3), Tb0, T0e);
    if i < size(traj,1)
        T_next = [reshape(traj(i+1,1:12), 3, 4); 0 0 0 1];
    else
        T_next = T;
    end
    [V, Xerr] = FeedbackControl(X, T, T_next, Kp, Ki, dt);

    % Arm Jacobian
    Blist = youBotArmSlist();
    Ja = JacobianBody(Blist, arm_joint_angles(:));
    if size(Ja,1) == 5 && size(Ja,2) == 6
        Ja = Ja';
    elseif size(Ja,1) ~= 6 || size(Ja,2) ~= 5
        error("JacobianBody returned unexpected shape: [%d x %d]", size(Ja,1), size(Ja,2));
    end

    % Fix wheel Jacobian F size (ensure 6x4)
    if size(Fraw,1) == 3 && size(Fraw,2) == 4
        F = [Fraw; zeros(3,4)];
    else
        error("Unexpected Fraw size: [%d x %d]", size(Fraw,1), size(Fraw,2));
    end

    % Combine into full Jacobian
    Je = [F, Ja];

    % Solve for joint/wheel speeds
    speeds = pinv(Je) * V;

    config = NextState(config, speeds, dt, max_speed);
    config_list = [config_list; config'];
    Xerr_log = [Xerr_log; Xerr'];
end

GenerateCSV('output.csv', config_list, gripper_state);
PlotError(Xerr_log);

%% --- LOCAL MANUAL FORWARD KINEMATICS OVERRIDE ---
function T0e = manualFK(thetalist)
    M = youBotArmHome();
    Blist = youBotArmSlist();
    T = eye(4);
    for i = length(thetalist):-1:1
        T = T * MatrixExp6(VecTose3(Blist(:, i) * thetalist(i)));
    end
    T0e = T * M;
end