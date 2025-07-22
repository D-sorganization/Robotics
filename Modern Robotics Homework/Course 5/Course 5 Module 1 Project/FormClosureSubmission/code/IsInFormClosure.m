function isFormClosure = IsInFormClosure(F)
% IsInFormClosure Determines if contact wrenches yield first-order form closure
%
% Input:
%   F: 3×j matrix (each column is a planar contact wrench)
% Output:
%   isFormClosure: true if the set of wrenches positively spans ℝ³, false otherwise
%
% Example:
%   F = [0 0 -1 2; -1 0 1 0; 0 -1 0 1];
%   isFormClosure = IsInFormClosure(F)

    [n, j] = size(F);
    if n ~= 3
        error('Input F must be a 3×j matrix of planar wrenches');
    end

    if rank(F) < 3
        isFormClosure = false;
        return;
    end

    f = ones(j,1);
    A = -eye(j);
    b = -ones(j,1);
    Aeq = F;
    beq = zeros(3,1);
    options = optimoptions('linprog','Display','none');
    [~, ~, exitflag] = linprog(f,A,b,Aeq,beq,[],[],options);

    isFormClosure = (exitflag == 1);
end
