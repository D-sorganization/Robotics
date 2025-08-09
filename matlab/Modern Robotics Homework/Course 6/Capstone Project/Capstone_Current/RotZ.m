function R = RotZ(theta)
% RotZ - Create rotation matrix for rotation about Z-axis
% This is equivalent to the Modern Robotics function
% Input:
%   theta - Rotation angle in radians
% Output:
%   R - 3x3 rotation matrix

ct = cos(theta);
st = sin(theta);

% Make almost zero elements exactly zero
if abs(st) < eps
    st = 0;
end
if abs(ct) < eps
    ct = 0;
end

R = [ct, -st, 0;
     st,  ct, 0;
     0,   0,  1];

end