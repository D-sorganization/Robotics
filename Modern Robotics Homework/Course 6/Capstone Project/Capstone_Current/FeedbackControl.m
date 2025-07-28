function [V, Xerr] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_history)
% FeedbackControl - Calculates commanded end-effector twist using PI control
% Inputs:
%   X - Current actual end-effector configuration (4x4 SE(3))
%   Xd - Current reference end-effector configuration (4x4 SE(3))
%   Xd_next - Reference configuration at next timestep (4x4 SE(3))
%   Kp - Proportional gain matrix (6x6)
%   Ki - Integral gain matrix (6x6)
%   dt - Timestep (s)
%   Xerr_history - Matrix of all previous error twists (6 x N-1)
%
% Outputs:
%   V - Commanded end-effector twist in end-effector frame (6x1)
%   Xerr - Current error twist (6x1)

% Calculate error twist
X_err = MatrixLog6(TransInv(X) * Xd);
Xerr = se3ToVec(X_err);

% Calculate integral error as sum of all previous errors (like working project)
if isempty(Xerr_history)
    Xerr_sum = zeros(6, 1);
else
    Xerr_sum = sum(Xerr_history, 2);
end

% Calculate feedforward reference twist
Vd_se3 = (1/dt) * MatrixLog6(TransInv(Xd) * Xd_next);
Vd = se3ToVec(Vd_se3);

% Calculate control law: V = [Ad(X^-1 * Xd)] * Vd + Kp * Xerr + Ki * integral_error
% This matches the working project implementation exactly
Ad_Xinv_Xd = Adjoint(TransInv(X) * Xd);
V = Ad_Xinv_Xd * Vd + Kp * Xerr + Ki * Xerr_sum * dt;

end

% Note: This code assumes the Modern Robotics library is in the MATLAB path
% All helper functions (MatrixLog6, TransInv, Adjoint, etc.) are from that library