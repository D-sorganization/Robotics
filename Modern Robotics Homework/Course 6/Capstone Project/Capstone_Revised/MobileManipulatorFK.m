function X = MobileManipulatorFK(q, theta, Slist, M, Tb0)
% Computes end-effector pose X of a mobile manipulator (KUKA youBot).
% Inputs:
%   q     - [phi; x; y] base configuration (radians and meters)
%   theta - 5x1 arm joint angles (radians)
%   Slist - 6x5 screw axes of arm in space frame
%   M     - 4x4 home configuration of end-effector
%   Tb0   - 4x4 transform from chassis frame {b} to arm base {0}

% Wheel-to-chassis transformation
phi = q(1); x = q(2); y = q(3);
T_sb = [cos(phi), -sin(phi), 0, x;
        sin(phi),  cos(phi), 0, y;
             0,         0, 1, 0.0963;
             0,         0, 0, 1];

% Forward kinematics of arm
T_0e = FKinSpace(M, Slist, theta);

% Combine transformations
X = T_sb * Tb0 * T_0e;
end
