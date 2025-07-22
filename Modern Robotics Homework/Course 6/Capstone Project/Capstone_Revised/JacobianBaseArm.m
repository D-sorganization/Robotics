function J = JacobianBaseArm(q, theta, Slist, Tb0, M)
% JacobianBaseArm: Computes the full Jacobian of the mobile manipulator
% Inputs:
%   q     - [phi; x; y] base configuration (radians, meters)
%   theta - 5x1 vector of joint angles
%   Slist - 6x5 screw axes of the arm in space frame
%   Tb0   - 4x4 transform from chassis frame {b} to arm base {0}
%   M     - 4x4 home configuration of the end-effector
% Output:
%   J     - 6x8 Jacobian (3 base + 5 arm DOF)

% Ensure theta is a column vector
theta = reshape(theta, [], 1);

% Constants for the chassis
r = 0.0475; l = 0.235; w = 0.15;

% Base Jacobian F matrix (3x4)
F = (r / 4) * [
    -1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w);
     1,        1,       1,        1;
    -1,        1,      -1,        1
];

% Extract chassis pose
phi = q(1); x = q(2); y = q(3);
R_sb = [cos(phi), -sin(phi), 0;
        sin(phi),  cos(phi), 0;
        0,         0,        1];
T_sb = [R_sb, [x; y; 0.0963];
         0 0 0 1];

% Convert F to 6x4 by appending zeros for rotation
F6 = [zeros(3,4);
       F];

% Forward kinematics to get T0e for adjoint transformation
T0e = FKinSpace(M, Slist, theta);

% Adjoint transformation from space frame to base of arm
Ad_T = Adjoint(inv(function J = JacobianBaseArm(q, theta, Slist, Tb0, M)
% JacobianBaseArm: Computes the full Jacobian of the mobile manipulator
% Inputs:
%   q     - [phi; x; y] base configuration (radians, meters)
%   theta - 5x1 vector of joint angles
%   Slist - 6x5 screw axes of the arm in space frame
%   Tb0   - 4x4 transform from chassis frame {b} to arm base {0}
%   M     - 4x4 home configuration of the end-effector
% Output:
%   J     - 6x8 Jacobian (3 base + 5 arm DOF)

% Ensure theta is a column vector
theta = reshape(theta, [], 1);

% Constants for the chassis
r = 0.0475; l = 0.235; w = 0.15;

% Base Jacobian F matrix (3x4)
F = (r / 4) * [
    -1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w);
     1,        1,       1,        1;
    -1,        1,      -1,        1
];

% Extract chassis pose
phi = q(1); x = q(2); y = q(3);
R_sb = [cos(phi), -sin(phi), 0;
        sin(phi),  cos(phi), 0;
        0,         0,        1];
T_sb = [R_sb, [x; y; 0.0963];
         0 0 0 1];

% Convert F to 6x4 by appending zeros for rotation
F6 = [zeros(3,4);
       F];

% Forward kinematics to get T0e for adjoint transformation
T0e = FKinSpace(M, Slist, theta);

% Adjoint transformation from space frame to base of arm
Ad_T = Adjoint(inv(T_sb * Tb0 * T0e));

% Compute base and arm Jacobians
J_base = Ad_T * F6(:, 1:3); % 6x3 base Jacobian
J_arm = JacobianSpace(Slist, theta); % 6x5 arm Jacobian

% Combine both Jacobians into full manipulator Jacobian
J = [J_base, J_arm];
end
T_sb * Tb0 * T0e));

% Compute base and arm Jacobians
J_base = Ad_T * F6(:, 1:3); % 6x3 base Jacobian
J_arm = JacobianSpace(Slist, theta); % 6x5 arm Jacobian

% Combine both Jacobians into full manipulator Jacobian
J = [J_base, J_arm];
end
