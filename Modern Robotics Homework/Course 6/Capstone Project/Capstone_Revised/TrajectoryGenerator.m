function traj = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k)
% Generates the trajectory for the end-effector through 8 motion segments.

traj = [];

segments = {
    {Tse_initial, Tsc_initial*Tce_standoff, 3},  % move to standoff
    {Tsc_initial*Tce_standoff, Tsc_initial*Tce_grasp, 1},  % descend
    {Tsc_initial*Tce_grasp, Tsc_initial*Tce_grasp, 1},     % grasp hold
    {Tsc_initial*Tce_grasp, Tsc_initial*Tce_standoff, 1},  % retract
    {Tsc_initial*Tce_standoff, Tsc_final*Tce_standoff, 3}, % move to final standoff
    {Tsc_final*Tce_standoff, Tsc_final*Tce_grasp, 1},      % descend
    {Tsc_final*Tce_grasp, Tsc_final*Tce_grasp, 1},         % release
    {Tsc_final*Tce_grasp, Tsc_final*Tce_standoff, 1}       % retract
};

for i = 1:length(segments)
    Xstart = segments{i}{1};
    Xend = segments{i}{2};
    Tf = segments{i}{3};
    N = Tf * k * 100;
    for j = 1:N
        s = (j-1)/(N-1);
        T = Xstart * MatrixExp6(MatrixLog6(TransInv(Xstart)*Xend)*s);
        traj = cat(3, traj, T);
    end
end
end

function se3mat = MatrixLog6(T)
[R, p] = TransToRp(T);
if norm(R - eye(3)) < 1e-6
    so3mat = zeros(3);
    se3mat = [so3mat, p; 0 0 0 0];
else
    acosinput = (trace(R)-1)/2;
    acosinput = max(min(acosinput,1),-1);
    theta = acos(acosinput);
    omgmat = theta/(2*sin(theta)) * (R - R');
    Ginv = eye(3)/theta - 0.5*omgmat + (1/theta - 0.5*cot(theta/2))*omgmat^2;
    v = Ginv * p;
    se3mat = [omgmat, v; 0 0 0 0];
end
end

function T = MatrixExp6(se3mat)
[omgmat, v] = se3ToVecSplit(se3mat);
theta = norm([omgmat(3,2); omgmat(1,3); omgmat(2,1)]);
if theta < 1e-6
    T = [eye(3), v; 0 0 0 1];
else
    omg = [omgmat(3,2); omgmat(1,3); omgmat(2,1)];
    omgmat = omgmat / theta;
    R = eye(3) + sin(theta)*omgmat + (1-cos(theta))*(omgmat^2);
    G = eye(3)*theta + (1 - cos(theta))*omgmat + (theta - sin(theta))*(omgmat^2);
    T = [R, G*v/theta; 0 0 0 1];
end
end

function [so3mat, v] = se3ToVecSplit(se3mat)
so3mat = se3mat(1:3,1:3);
v = se3mat(1:3,4);
end

function Tinv = TransInv(T)
[R, p] = TransToRp(T);
Tinv = [R', -R'*p; 0 0 0 1];
end

function [R, p] = TransToRp(T)
R = T(1:3,1:3);
p = T(1:3,4);
end
