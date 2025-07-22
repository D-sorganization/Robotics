% Full Project Simulation Script

% Initial chassis and arm configuration
q_init = [0; 0; 0];
theta_init = zeros(5,1);

% Robot parameters
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

% Trajectory parameters
Tse_initial = [1 0 0 0; 0 1 0 0; 0 0 1 0.5; 0 0 0 1];
Tsc_initial = [1 0 0 1; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Tsc_final = [0 1 0 0; -1 0 0 -1; 0 0 1 0; 0 0 0 1];
Tce_grasp = [cos(pi/4) 0 sin(pi/4) 0; 0 1 0 0; -sin(pi/4) 0 cos(pi/4) 0; 0 0 0 1];
Tce_standoff = [cos(pi/4) 0 sin(pi/4) 0; 0 1 0 0; -sin(pi/4) 0 cos(pi/4) 0.1; 0 0 0 1];

k = 1; % Trajectory scaling
traj = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k);

% Control gains
Kp = 5.0 * eye(6);
Ki = 0.1 * eye(6);

% Simulation timestep
dt = 0.01;

% Run simulation
SimulationRunner(q_init, theta_init, Slist, M, Tb0, traj, Kp, Ki, dt);