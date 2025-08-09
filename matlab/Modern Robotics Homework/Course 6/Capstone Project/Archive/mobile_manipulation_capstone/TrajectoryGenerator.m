function [traj, gripper_state] = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff)
k = 1;
Tf_move = 3;
Tf_grasp = 0.625;
dt = 0.01;
Tse_standoff_init = Tsc_initial * Tce_standoff;
Tse_grasp = Tsc_initial * Tce_grasp;
Tse_standoff_goal = Tsc_goal * Tce_standoff;
Tse_release = Tsc_goal * Tce_grasp;
traj = [];
gripper_state = [];
traj1 = TrajCellToTensor(ScrewTrajectory(Tse_initial, Tse_standoff_init, Tf_move, Tf_move/dt, 5));
gripper1 = zeros(size(traj1,3),1);
traj2 = TrajCellToTensor(ScrewTrajectory(Tse_standoff_init, Tse_grasp, Tf_move/2, (Tf_move/2)/dt, 5));
gripper2 = zeros(size(traj2,3),1);
traj3 = zeros(4,4,round(Tf_grasp/dt));
for i = 1:size(traj3,3)
    traj3(:,:,i) = Tse_grasp;
end
gripper3 = ones(size(traj3,3),1);
traj4 = TrajCellToTensor(ScrewTrajectory(Tse_grasp, Tse_standoff_init, Tf_move/2, (Tf_move/2)/dt, 5));
gripper4 = ones(size(traj4,3),1);
traj5 = TrajCellToTensor(ScrewTrajectory(Tse_standoff_init, Tse_standoff_goal, Tf_move, Tf_move/dt, 5));
gripper5 = ones(size(traj5,3),1);
traj6 = TrajCellToTensor(ScrewTrajectory(Tse_standoff_goal, Tse_release, Tf_move/2, (Tf_move/2)/dt, 5));
gripper6 = ones(size(traj6,3),1);
traj7 = zeros(4,4,round(Tf_grasp/dt));
for i = 1:size(traj7,3)
    traj7(:,:,i) = Tse_release;
end
gripper7 = zeros(size(traj7,3),1);
traj8 = TrajCellToTensor(ScrewTrajectory(Tse_release, Tse_standoff_goal, Tf_move/2, (Tf_move/2)/dt, 5));
gripper8 = zeros(size(traj8,3),1);
traj_total = cat(3, traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8);
gripper_total = [gripper1; gripper2; gripper3; gripper4; gripper5; gripper6; gripper7; gripper8];
for i = 1:size(traj_total,3)
    T = traj_total(:,:,i);
    traj = [traj; reshape(T(1:3,1:4),1,12)];
end
gripper_state = gripper_total;
end