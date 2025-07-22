function thetamatrix = Project3Function(thetalist0,dthetalist0,N,EndTime,taulist,Ftip0,Mlist,Glist,Slist,g)
% M01 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.089159; 0, 0, 0, 1];
% M12 = [0, 0, 1, 0.28; 0, 1, 0, 0.13585; -1, 0, 0, 0; 0, 0, 0, 1];
% M23 = [1, 0, 0, 0; 0, 1, 0, -0.1197; 0, 0, 1, 0.395; 0, 0, 0, 1];
% M34 = [0, 0, 1, 0; 0, 1, 0, 0; -1, 0, 0, 0.14225; 0, 0, 0, 1];
% M45 = [1, 0, 0, 0; 0, 1, 0, 0.093; 0, 0, 1, 0; 0, 0, 0, 1];
% M56 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.09465; 0, 0, 0, 1];
% M67 = [1, 0, 0, 0; 0, 0, 1, 0.0823; 0, -1, 0, 0; 0, 0, 0, 1];
% G1 = diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7]);
% G2 = diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393]);
% G3 = diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275]);
% G4 = diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219]);
% G5 = diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219]);
% G6 = diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879]);
% Glist = cat(3, G1, G2, G3, G4, G5, G6);
% Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67); 
% Slist = [0,         0,         0,         0,        0,        0;
%          0,         1,         1,         1,        0,        1;
%          1,         0,         0,         0,       -1,        0;
%          0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491;
%          0,         0,         0,         0,  0.81725,        0;
%          0,         0,     0.425,   0.81725,        0,  0.81725];

% Problem Specific Parameters

% Initial Position 
thetalist=thetalist0;

% Initial Velocity
dthetalist=dthetalist0;

% Wrench at Tip
Ftip = Ftip0;

% % Gravity
% g = [0,0,-9.81];

% % Total Number of Iterations
% N = 500;

% Differential Time Per Iteration in Seconds
dt = EndTime/N;

% Taulist is the Input Torques to the robot. Since the input torques are
% all zero, so is the taulist.
taulist = [0,0,0,0,0,0]';

% Define a blank matrix to fill in with the taulist for each 
thetamatrix =[thetalist0];

for j = 1:N
    % Compute acceleration using ForwardDynamics
    ddthetalist = ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist);
    % Compute joint positions using initial position and product of
    % velocity times time. Assume linearity over the step
    thetalist = thetalist+dt*dthetalist;
    % Compute velocity using initial velocity and product of acceleration
    % and time.
    dthetalist = dthetalist+dt*ddthetalist;
    % Add the theta values for the timestep to the thetamatrix.
    thetamatrix = [thetamatrix,thetalist];
end

% Write thetamatrix to a CSV file. The transpose is used on theta so that
% the format is correct for CoppeliaSim
cd(matlabdrive);
cd '_Modern Robotics Course/Homework/Course 3';
% csvwrite('Simulation1.csv',thetamatrix');
csvwrite('Simulation2.csv',thetamatrix');

end