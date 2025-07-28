function T = ArmFK(theta, Slist, M)
% ArmFK computes the forward kinematics of the manipulator arm using
% the Product of Exponentials formula.
%
% Inputs:
%   theta: Vector of joint angles [theta1; theta2; ...; theta_n]
%   Slist: Screw axes of the arm joints (6xn matrix)
%   M: Home configuration of the end-effector (4x4 matrix)
%
% Output:
%   T: Homogeneous transformation of the end-effector relative to arm base

T = eye(4);
for i = 1:length(theta)
    T = T * expm(VecTose3(Slist(:, i)) * theta(i));
end

T = T * M;

end

% Helper function
function se3mat = VecTose3(V)
% Converts a spatial velocity vector into an se(3) representation
se3mat = [  0,   -V(3),  V(2), V(4);
          V(3),   0,   -V(1), V(5);
         -V(2), V(1),    0,   V(6);
             0,    0,     0,     0];
end
