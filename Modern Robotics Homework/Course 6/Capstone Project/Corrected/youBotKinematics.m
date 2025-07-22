function [T0e, Je] = youBotKinematics(config)
% youBotKinematics - Computes forward kinematics and Jacobian for youBot
% Input:
%   config - 12x1 configuration vector [phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4]
% Outputs:
%   T0e - End-effector configuration in space frame (4x4)
%   Je - Full Jacobian of the robot (6x9)

% Extract configuration components
phi = config(1);
x = config(2);
y = config(3);
arm_angles = config(4:8);

% Robot parameters
Tb0 = [1, 0, 0, 0.1662;
       0, 1, 0, 0;
       0, 0, 1, 0.0026;
       0, 0, 0, 1];

M0e = [1, 0, 0, 0.033;
       0, 1, 0, 0;
       0, 0, 1, 0.6546;
       0, 0, 0, 1];

% Screw axes in end-effector frame at home configuration
Blist = [0,  0,  1,  0,      0.033, 0;
         0, -1,  0, -0.5076, 0,     0;
         0, -1,  0, -0.3526, 0,     0;
         0, -1,  0, -0.2176, 0,     0;
         0,  0,  1,  0,      0,     0]';

% Chassis transformation
Tsb = [cos(phi), -sin(phi), 0, x;
       sin(phi),  cos(phi), 0, y;
       0,         0,        1, 0.0963;
       0,         0,        0, 1];

% Forward kinematics of arm
T0e_in_0 = FKinBody(M0e, Blist, arm_angles);

% End-effector in space frame
T0e = Tsb * Tb0 * T0e_in_0;

% Compute Jacobian if requested
if nargout > 1
    % Arm Jacobian in end-effector frame
    Jarm = JacobianBody(Blist, arm_angles);
    
    % Base Jacobian
    r = 0.0475;  % wheel radius
    l = 0.235;   % half-length
    w = 0.15;    % half-width
    
    % F matrix for mecanum wheels (in body frame)
    F = (r/4) * [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w);
                  1,        1,       1,        1;
                 -1,        1,      -1,        1];
    
    % Extend F to 6x4 (add zeros for angular velocities)
    F6 = [zeros(3,4); F];
    
    % Transform base Jacobian to end-effector frame
    Teb = TransInv(T0e_in_0) * TransInv(Tb0);
    Jbase = Adjoint(Teb) * F6;
    
    % Combined Jacobian: [base_wheels, arm_joints]
    Je = [Jbase, Jarm];
end

end

% Note: This code assumes the Modern Robotics library is in the MATLAB path
% All helper functions (FKinBody, JacobianBody, etc.) are from that library