function testJointLimits = checkJointLimits(arm_angles)
% checkJointLimits - Check if arm configuration violates joint limits
% Input:
%   arm_angles - 5x1 vector of arm joint angles
% Output:
%   testJointLimits - Vector of violated joint indices (empty if none)

    % Conservative joint limits to avoid singularities and self-collision
    % Joint 3 and 4 should not be too close to zero (singularity)
    % Joint 2 should not go too far back (self-collision with base)
    joint_limits = [
        -2.9, 2.9;      % Joint 1
        -1.5, 1.0;      % Joint 2 (limited backward motion)
        -2.2, -0.1;     % Joint 3 (avoid zero)
        -1.8, -0.1;     % Joint 4 (avoid zero)
        -2.9, 2.9       % Joint 5
    ];
    
    testJointLimits = [];
    for i = 1:5
        if arm_angles(i) < joint_limits(i,1) || arm_angles(i) > joint_limits(i,2)
            testJointLimits = [testJointLimits, i];
        end
    end
end