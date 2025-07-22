function isFormClosure = EvaluateFormClosure(F)
% IsInFormClosure Determines if contact wrenches yield first-order form closure
%
% Input:
%   F: 3×j matrix (each column is a planar contact wrench)
% Output:
%   isFormClosure: true if the set of wrenches positively spans ℝ³, false otherwise

    % Size check
    [n, j] = size(F);
    if n ~= 3
        error('Input F must be a 3×j matrix of planar wrenches');
    end

    % Rank condition: must span ℝ³
    if rank(F) < 3
        isFormClosure = false;
        return;
    end

    % LP to check for strictly positive k such that Fk = 0
    f = ones(j,1);                         % Dummy objective
    A = -eye(j);                           % Enforce k ≥ 1
    b = -ones(j,1);
    Aeq = F;                               % Fk = 0
    beq = zeros(3,1);
    options = optimoptions('linprog','Display','none');
    [~, ~, exitflag] = linprog(f,A,b,Aeq,beq,[],[],options);

    isFormClosure = (exitflag == 1);
end
