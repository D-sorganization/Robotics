function Je_limited = applyJointLimits(Je, violated_joints)
% applyJointLimits - Zero out columns of Jacobian for joints at limits
% Inputs:
%   Je - 6x9 Jacobian matrix
%   violated_joints - Indices of joints that violate limits
% Output:
%   Je_limited - Modified Jacobian with zeroed columns

    Je_limited = Je;
    for i = violated_joints
        Je_limited(:, 4+i) = 0;  % Arm joints are columns 5-9
    end
end