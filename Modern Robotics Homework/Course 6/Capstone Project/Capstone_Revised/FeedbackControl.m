function V = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, integral_error)
% FeedbackControl computes control velocities using a combined feedforward
% and proportional-integral feedback controller for trajectory tracking.
%
% Inputs:
%   X: Current end-effector configuration (4x4 matrix)
%   Xd: Desired end-effector configuration at current timestep (4x4)
%   Xd_next: Desired configuration at next timestep (4x4)
%   Kp, Ki: Proportional and integral gains (6x6 matrices)
%   dt: Timestep duration
%   integral_error: Accumulated integral error (6x1 vector)
%
% Output:
%   V: Spatial velocity command (6x1 vector)

X_err = se3ToVec(MatrixLog6(TransInv(X) * Xd));
integral_error = integral_error + X_err * dt;

Vd = se3ToVec(MatrixLog6(TransInv(Xd) * Xd_next) / dt);

Ad_term = Adjoint(TransInv(X) * Xd);
V = Ad_term * Vd + Kp * X_err + Ki * integral_error;

end

% Helper functions
function AdT = Adjoint(T)
[R,p] = TransToRp(T);
AdT = [R, zeros(3); VecToso3(p)*R, R];
end

function so3mat = VecToso3(omg)
so3mat = [  0,   -omg(3), omg(2);
         omg(3),    0,   -omg(1);
        -omg(2), omg(1),    0];
end

function vec = se3ToVec(se3mat)
vec = [se3mat(3,2); se3mat(1,3); se3mat(2,1); se3mat(1:3,4)];
end

function Tinv = TransInv(T)
[R,p] = TransToRp(T);
Tinv = [R', -R'*p; 0,0,0,1];
end

function [R,p] = TransToRp(T)
R = T(1:3,1:3);
p = T(1:3,4);
end

function se3mat = MatrixLog6(T)
[R,p] = TransToRp(T);
if norm(R-eye(3)) < 1e-5
    se3mat = [zeros(3), p; 0, 0, 0, 0];
else
    acosinput = (trace(R)-1)/2;
    theta = acos(max(min(acosinput,1),-1));
    so3mat = theta / (2*sin(theta)) * (R-R');
    se3mat = [so3mat, p; 0, 0, 0, 0];
end
end
