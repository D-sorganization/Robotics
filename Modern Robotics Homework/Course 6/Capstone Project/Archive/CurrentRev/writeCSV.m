function writeCSV(filename, configurations, gripper_states)
% writeCSV - Write robot configurations to CSV file for CoppeliaSim
% Inputs:
%   filename - Output CSV filename
%   configurations - Nx12 matrix of robot configurations
%   gripper_states - Nx1 vector of gripper states (0=open, 1=closed)

    N = size(configurations, 1);
    data = [configurations, gripper_states];
    
    % Write to CSV with appropriate precision
    dlmwrite(filename, data, 'delimiter', ',', 'precision', '%.6f');
    fprintf('CSV file written: %s (%d configurations)\n', filename, N);
end