% *** This script implements the mobile manipulation capstone project *** %
% *** Project website: 
% http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone

% Including : a (screw or straight line motion only)trajectory generater
%             a kinematic simulator to update current configuration
%             (first-order Euler-step method)
%             a feedforward and PI feedbackward velocity controller in the
%             task space.
% Introduction : This script use the two Class file "robot_base.m" and "
%                mobile_manipulator.m" which is inherited from its superclass to 
%                conclude and generate all the useful function and 
%                parameter needed. 
%                The code first generate the end-effector trajectory then 
%                control the robot to follow that trajectory from a
%                differnt initial configuration.

% Add the Functions library to path
addpath(genpath('Functions'));

% Create results folder
if ~exist('results', 'dir')
    mkdir('results');
end

% mobile base parameters
l = 0.47/2;
w = 0.3/2;
r = 0.0475;
T_b0 = RpToTrans(eye(3),[0.1662,0,0.0026]');
M_0e = RpToTrans(eye(3),[0.033,0,0.6546]');
Blists = [[0,0,1,0,0.033,0]',[0,-1,0,-0.5076,0,0]',[0,-1,0,-0.3526,0,0]'...
    [0,-1,0,-0.2176,0,0]',[0,0,1,0,0,0]'];

% cube configuration
T_sc_initial = RpToTrans(eye(3),[1,0,0.025]');
T_sc_goal = RpToTrans(rotz(-pi/2),[0,-1,0.025]');
a = pi/5;
T_ce_standoff = [[-sin(a),0,-cos(a),0]',[0,1,0,0]',[cos(a),0,-sin(a),0]',...
    [0,0,0.25,1]'];
T_ce_grasp = [[-sin(a),0,-cos(a),0]',[0,1,0,0]',[cos(a),0,-sin(a),0]',...
    [0,0,0,1]'];
% end-effector planned configuration(reference) 
T_se_initial = [0,0,1,0;0,1,0,0;-1,0,0,0.5;0,0,0,1];
T_standoff_initial = T_sc_initial * T_ce_standoff;
T_grasp = T_sc_initial * T_ce_grasp;
T_standoff_final = T_sc_goal * T_ce_standoff;
T_release = T_sc_goal * T_ce_grasp;

%Construct a cell array for the path
T_configure = {T_se_initial,T_standoff_initial,T_grasp,T_grasp,...
    T_standoff_initial,T_standoff_final,T_release,T_release,...
    T_standoff_final};
disp('Initialization completed');

% Create the robot object Mybot
Mybot = mobile_manipulator(l,w,r,T_b0,M_0e,Blists);
disp('Robot object created!');
% Generating reference trajectory
dt = 0.01;% manual choose 
T_total = 13;% manual choose the total motion time 
Tf = calculateTf(T_total);% calculate the weighted time for each piece
Traj = [];% N * 13 matrix, N is the number of reference frame
grasp_state = 0;
for i = 1:8
    if i == 3 
        grasp_state = 1;
    elseif i == 7
        grasp_state = 0;
    end
    
    Trajectory = Mybot.TrajectoryGenerator(T_configure{i},...
        T_configure{i+1},Tf(i),dt,grasp_state,'Cartesian',5);
    Traj = [Traj;Trajectory];
end
writematrix(Traj,'results/Traj.csv');
disp('Trajectory generated');

% Manual choose the initial parameter of the controller and the robot
% configuration
Mybot.q = [0,0,0];
Mybot.theta = [0,0,0.2,-1.67,0]';
Mybot.wheelAngle = [pi/2,pi/2,pi/2,pi/2];% Do not affect the simulation
Mybot.kp = 7.0 * eye(6);  
Mybot.ki = 0.4 * eye(6);  
maxspeed = 12.3*ones(1,9);% take uniform max speed.
jointLimits = [[pi,-pi]',[pi,-pi]',...
    [pi,-pi]',[pi,-pi]'...
    ,[pi,-pi]'];% Designed values to avoid collision : [max;min]

[Td,grasp] = traj2mat(Traj);% Td is a 3D matrix

% Apply Fb control to the Robot object.
% All other parameter needed is included in the obj's property.
[Animation,Xerr] = Mybot.FeedbackControl(dt,Td,maxspeed,grasp,jointLimits);
disp('Feedback Control applied, Animation file generated');

% plot the error twist between the reference configuration and current
% configurtaion: to see the control system performance.
p = plot(Xerr','LineWidth',1.5);
title('Xerr versus time','FontSize',12,'FontWeight','bold')
xlabel('Time (0.01s)','FontSize',12,'FontWeight','bold');
ylabel('Error Twist','FontSize',12,'FontWeight','bold');
legend(p,'Wx','Wy','Wz','Vx','Vy','Vz')
grid on;

% Save required output files in results folder
writematrix(Animation,'results/Animation.csv');
save('results/Xerr.mat','Xerr');

% Create additional analysis plots
figure('Name', 'Simulation Analysis', 'Position', [200, 200, 1200, 800]);

% Plot 1: Error magnitude over time
subplot(2, 3, 1);
error_magnitude = sqrt(sum(Xerr.^2, 1));
plot(error_magnitude, 'LineWidth', 2, 'Color', 'red');
title('Total Error Magnitude');
xlabel('Time Steps');
ylabel('Error Magnitude');
grid on;

% Plot 2: End-effector trajectory
subplot(2, 3, 2);
time_steps = 1:size(Animation, 1);
plot(time_steps, Animation(:, 4), 'b-', 'LineWidth', 2, 'DisplayName', 'X');
hold on;
plot(time_steps, Animation(:, 5), 'r-', 'LineWidth', 2, 'DisplayName', 'Y');
plot(time_steps, Animation(:, 6), 'g-', 'LineWidth', 2, 'DisplayName', 'Z');
title('End-Effector Position');
xlabel('Time Steps');
ylabel('Position (m)');
legend('Location', 'best');
grid on;

% Plot 3: Joint angles
subplot(2, 3, 3);
plot(time_steps, Animation(:, 7), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Joint 1');
hold on;
plot(time_steps, Animation(:, 8), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Joint 2');
plot(time_steps, Animation(:, 9), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Joint 3');
plot(time_steps, Animation(:, 10), 'm-', 'LineWidth', 1.5, 'DisplayName', 'Joint 4');
plot(time_steps, Animation(:, 11), 'c-', 'LineWidth', 1.5, 'DisplayName', 'Joint 5');
title('Joint Angles');
xlabel('Time Steps');
ylabel('Angle (rad)');
legend('Location', 'best');
grid on;

% Plot 4: Mobile base trajectory
subplot(2, 3, 4);
plot(Animation(:, 2), Animation(:, 3), 'b-', 'LineWidth', 2);
title('Mobile Base Trajectory');
xlabel('X Position (m)');
ylabel('Y Position (m)');
grid on;
axis equal;

% Plot 5: Grasp state
subplot(2, 3, 5);
plot(time_steps, Animation(:, 13), 'LineWidth', 2, 'Color', 'green');
title('Grasp State');
xlabel('Time Steps');
ylabel('Grasp (0/1)');
ylim([-0.1, 1.1]);
grid on;

% Plot 6: Performance metrics
subplot(2, 3, 6);
final_error = sqrt(sum(Xerr(:, end).^2));
max_error = max(sqrt(sum(Xerr.^2, 1)));
mean_error = mean(sqrt(sum(Xerr.^2, 1)));

metrics = [final_error, max_error, mean_error];
metric_names = {'Final Error', 'Max Error', 'Mean Error'};
bar(metrics, 'FaceColor', [0.3, 0.6, 0.9]);
title('Performance Metrics');
ylabel('Error Magnitude');
set(gca, 'XTickLabel', metric_names);
grid on;

% Add text annotations
for i = 1:length(metrics)
    text(i, metrics(i) + max(metrics)*0.05, sprintf('%.4f', metrics(i)), ...
         'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end

sgtitle('Mobile Manipulation Simulation Results', 'FontSize', 14, 'FontWeight', 'bold');

% Save the analysis plot
saveas(gcf, 'results/simulation_analysis.png');
saveas(gcf, 'results/simulation_analysis.pdf');

% Create performance summary
fid = fopen('results/performance_summary.txt', 'w');
fprintf(fid, 'Mobile Manipulation Simulation Results\n');
fprintf(fid, '=====================================\n\n');
fprintf(fid, 'Control Parameters:\n');
fprintf(fid, 'Kp = %.1f\n', 1.0);
fprintf(fid, 'Ki = %.1f\n\n', 0.4);
fprintf(fid, 'Performance Metrics:\n');
fprintf(fid, 'Final Error: %.6f\n', final_error);
fprintf(fid, 'Max Error: %.6f\n', max_error);
fprintf(fid, 'Mean Error: %.6f\n', mean_error);
fprintf(fid, 'Simulation Steps: %d\n', size(Animation, 1));
fclose(fid);

disp('Successfully plot Xerror versus time');
disp('All results saved to results/ folder');
disp('Files generated:');
disp('  - results/Animation.csv');
disp('  - results/Traj.csv');
disp('  - results/Xerr.mat');
disp('  - results/simulation_analysis.png');
disp('  - results/simulation_analysis.pdf');
disp('  - results/performance_summary.txt');

% Clean up workspace
disp('Cleaning up workspace...');
clear l w r T_b0 M_0e Blists T_sc_initial T_sc_goal a T_ce_standoff T_ce_grasp;
clear T_se_initial T_standoff_initial T_grasp T_standoff_final T_release;
clear T_configure Mybot dt T_total Tf Traj grasp_state Trajectory;
clear maxspeed jointLimits Td grasp Animation Xerr;
clear error_magnitude time_steps final_error max_error mean_error;
clear metrics metric_names fid p;
disp('Workspace cleanup complete.');
