function Tse = MobileManipulatorFK(q, theta, Slist, M, Tb0)
% MobileManipulatorFK computes the forward kinematics for the mobile
% manipulator, combining chassis and arm configurations.
%
% Inputs:
%   q: Chassis configuration [phi; x; y]
%   theta: Arm joint angles [theta1; theta2; ...; theta_n]
%   Slist: Screw axes of the arm joints (6xn matrix)
%   M: Home configuration of end-effector relative to arm base (4x4)
%   Tb0: Transformation from chassis frame {b} to arm base frame {0} (4x4)
%
% Output:
%   Tse: Homogeneous transformation of end-effector relative to global frame

phi = q(1);
x = q(2);
y = q(3);

% Transformation from space frame to chassis frame
Tsb = [cos(phi), -sin(phi), 0, x;
       sin(phi),  cos(phi), 0, y;
            0,        0,    1, 0;
            0,        0,    0, 1];

% Arm forward kinematics
T0e = eye(4);
for i = 1:length(theta)
    T0e = T0e * expm(VecTose3(Slist(:, i)) * theta(i));
end
T0e = T0e * M;

% Combined transformation
Tse = Tsb * Tb0 * T0e;

end

% Helper function
function se3mat = VecTose3(V)
% Converts a spatial velocity vector into an se(3) representation
se3mat = [  0,   -V(3),  V(2), V(4);
          V(3),   0,   -V(1), V(5);
         -V(2), V(1),    0,   V(6);
             0,    0,     0,     0];
end
