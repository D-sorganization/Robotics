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

% Forward kinematics of arm in base frame {0}
T0e_in_0 = FKinBody(M0e, Blist, arm_angles);

% Full transformation: body frame to end-effector
Tbe = Tb0 * T0e_in_0;

% End-effector in space frame
T0e = Tsb * Tbe;

% Compute Jacobian if requested
if nargout > 1
    % Arm Jacobian in end-effector frame
    Jarm = JacobianBody(Blist, arm_angles);
    
    % Base parameters - use correct dimensions from working project
    r = 0.0475;  % wheel radius
    l = 0.47/2;  % half-length (matching working project exactly)
    w = 0.3/2;   % half-width (matching working project exactly)
    
    % H_0 matrix (same as working project)
    H_0 = (1/r) * [-l-w, 1, -1;
                    l+w, 1,  1;
                    l+w, 1, -1;
                   -l-w, 1,  1];
    
    % Calculate F as pseudoinverse of H_0 (key fix from working project)
    F = pinv(H_0, 0.0001);
    
    % Create F6 matrix (6x4) - correct format from working project
    F6 = [zeros(2,4);  % First two rows zeros
          F;           % F matrix (3x4)
          zeros(1,4)]; % Last row zeros
    
    % Transform base Jacobian to end-effector frame
    % Use transformation from end-effector to body frame
    Teb = TransInv(Tbe);
    Jbase = Adjoint(Teb) * F6;
    
    % Combined Jacobian: [base_wheels, arm_joints]
    Je = [Jbase, Jarm];
end

end

% Note: This code assumes the Modern Robotics library is in the MATLAB path
% All helper functions (FKinBody, JacobianBody, etc.) are from that library