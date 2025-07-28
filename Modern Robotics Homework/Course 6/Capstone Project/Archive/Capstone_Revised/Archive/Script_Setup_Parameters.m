% Initial configuration
q_init = [0; 0; 0];
theta_init = zeros(5,1);

% Robot specific parameters (explicitly defined)
Slist = [0, 0, 0, 0, 0;
         0, 1, 1, 1, 0;
         1, 0, 0, 0, 1;
         0, 0.033, 0.033, 0.033, 0;
         0, 0, 0, 0, 0;
         0, 0, 0, 0.2, 0.2];

M = [1, 0, 0, 0.0330;
     0, 1, 0, 0;
     0, 0, 1, 0.6546;
     0, 0, 0, 1];

Tb0 = [1, 0, 0, 0.1662;
       0, 1, 0, 0;
       0, 0, 1, 0.0026;
       0, 0, 0, 1];

% Desired trajectory generation
Xstart = eye(4);
Xend = [0, 0, 1, 0.5;
        0, 1, 0, 0;
       -1, 0, 0, 0.5;
        0, 0, 0, 1];
Tf = 5;  
N = 100; 
traj = TrajectoryGenerator(Xstart, Xend, Tf, N);

% Control gains
Kp = 5.0 * eye(6);  % recommended initial tuning
Ki = 0.1 * eye(6);  % recommended initial tuning

% Run simulation
dt = Tf/(N-1);
SimulationRunner(q_init, theta_init, Slist, M, Tb0, traj, Kp, Ki, dt);
