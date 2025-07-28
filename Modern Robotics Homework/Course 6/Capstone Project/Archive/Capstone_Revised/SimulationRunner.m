function SimulationRunner(q_init, theta_init, Slist, M, Tb0, traj, Kp, Ki, dt)
% SimulationRunner runs the full simulation for the mobile manipulator.
%
% Inputs:
%   q_init: Initial chassis configuration [phi; x; y]
%   theta_init: Initial joint angles
%   Slist: Screw axes of arm joints
%   M: End-effector home configuration
%   Tb0: Transform from chassis frame to arm base
%   traj: Desired trajectory (4x4xN array)
%   Kp, Ki: Controller gain matrices
%   dt: Timestep

q = q_init;
theta = theta_init;
N = size(traj, 3);
integral_error = zeros(6,1);

for i = 1:N-1
    X = MobileManipulatorFK(q, theta, Slist, M, Tb0);
    Xd = traj(:,:,i);
    Xd_next = traj(:,:,i+1);

    V = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, integral_error);
    integral_error = integral_error + se3ToVec(MatrixLog6(TransInv(X) * Xd)) * dt;

    % Extract wheel and joint velocities
    % Assuming 4 wheels + n arm joints
    u = PseudoInverseJacobian(q, theta, Slist, Tb0, M) * V;

    % Update configurations
    q = ChassisOdometry(q, u(1:4), dt);
    theta = theta + u(5:end)*dt;

    % Plot and visualization step (optional but recommended)
    % plotCurrentConfiguration(q, theta, Slist, M, Tb0);
    % pause(0.01);
end

end

% Helper functions (use previously defined implementations or adjust as needed)

function Jpseudo = PseudoInverseJacobian(q, theta, Slist, Tb0, M)
% Computes pseudoinverse of the manipulator Jacobian (base+arm)
Tse = MobileManipulatorFK(q, theta, Slist, M, Tb0);
J = JacobianBaseArm(q, theta, Slist, Tb0, M);
Jpseudo = pinv(J);
end

function J = JacobianBaseArm(q, theta, Slist, Tb0, M)
% Computes Jacobian of the mobile manipulator (base + arm)
% To be implemented based on manipulator and base configurations
% This function requires the Jacobian calculation as per your specific system
end
