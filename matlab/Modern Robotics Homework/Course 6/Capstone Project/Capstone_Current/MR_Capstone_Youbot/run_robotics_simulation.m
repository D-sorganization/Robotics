function [Animation, Xerr, Traj, Td, grasp] = run_robotics_simulation(params)
    % Standalone function to run robotics simulation with given parameters
    % Input: params - struct with kp, ki, dt, T_total, maxspeed
    % Output: Animation, Xerr, Traj, Td, grasp
    
    % Add the Functions library to path
    addpath(genpath('Functions'));
    
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
    
    % Create the robot object Mybot
    Mybot = mobile_manipulator(l,w,r,T_b0,M_0e,Blists);
    
    % Generating reference trajectory
    dt = params.dt;
    T_total = params.T_total;
    Tf = calculateTf(T_total, T_se_initial, T_standoff_initial, T_grasp, T_standoff_final, T_release);
    Traj = [];
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
    
    % Set controller parameters
    Mybot.q = [0,0,0];
    Mybot.theta = [0,0,0.2,-1.67,0]';
    Mybot.wheelAngle = [pi/2,pi/2,pi/2,pi/2];
    Mybot.kp = params.kp * eye(6);
    Mybot.ki = params.ki * eye(6);
    maxspeed = params.maxspeed * ones(1,9);
    jointLimits = [[pi,-pi]',[pi,-pi]',[pi,-pi]',[pi,-pi]',[pi,-pi]'];
    
    [Td,grasp] = traj2mat(Traj);
    
    % Apply feedback control
    [Animation,Xerr] = Mybot.FeedbackControl(dt,Td,maxspeed,grasp,jointLimits);
    
    % Clean up temporary variables (keep only output variables)
    clear l w r T_b0 M_0e Blists T_sc_initial T_sc_goal a T_ce_standoff T_ce_grasp;
    clear T_se_initial T_standoff_initial T_grasp T_standoff_final T_release;
    clear T_configure Mybot dt T_total Tf grasp_state Trajectory;
    clear maxspeed jointLimits;
end
