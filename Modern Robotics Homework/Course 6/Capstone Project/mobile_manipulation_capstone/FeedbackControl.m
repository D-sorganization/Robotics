function [V, Xerr] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt)
persistent integral_error
if isempty(integral_error)
    integral_error = zeros(6,1);
end
Vd_se3 = MatrixLog6(TransInv(Xd) * Xd_next);
Vd = se3ToVec(Vd_se3) / dt;
Xerr_se3 = MatrixLog6(TransInv(X) * Xd);
Xerr = se3ToVec(Xerr_se3);
integral_error = integral_error + Xerr * dt;
V = Adjoint(TransInv(X) * Xd) * Vd + Kp * Xerr + Ki * integral_error;
end