function T = RpToTrans(R, p)
% RpToTrans - Create transformation matrix from rotation and position
% This is a standard Modern Robotics function
% Inputs:
%   R - 3x3 rotation matrix
%   p - 3x1 position vector
% Output:
%   T - 4x4 transformation matrix

T = [R, p; 0 0 0 1];

end