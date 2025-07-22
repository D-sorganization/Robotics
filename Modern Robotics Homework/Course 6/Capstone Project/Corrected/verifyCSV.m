function verifyCSV(filename)
% verifyCSV - Verify that a CSV file has the correct format for CoppeliaSim
% Input:
%   filename - Name of CSV file to verify

if nargin < 1
    filename = 'Animation.csv';
end

fprintf('\nVerifying CSV file: %s\n', filename);
fprintf('=====================================\n');

% Check if file exists
if ~exist(filename, 'file')
    fprintf('✗ ERROR: File not found!\n');
    return;
end

% Load the CSV file
try
    data = readmatrix(filename);
catch
    fprintf('✗ ERROR: Could not read file as numeric matrix!\n');
    return;
end

% Check dimensions
[rows, cols] = size(data);
fprintf('File dimensions: %d rows x %d columns\n', rows, cols);

if cols ~= 13
    fprintf('✗ ERROR: File should have 13 columns, but has %d\n', cols);
    fprintf('Expected format: [phi, x, y, J1-J5, W1-W4, gripper]\n');
    return;
else
    fprintf('✓ Correct number of columns (13)\n');
end

if rows < 100
    fprintf('⚠ WARNING: Only %d configurations (expected >1000 for full trajectory)\n', rows);
else
    fprintf('✓ Sufficient number of configurations (%d)\n', rows);
end

% Check data ranges
fprintf('\nData ranges:\n');
fprintf('  phi (chassis angle): [%.3f, %.3f] rad\n', min(data(:,1)), max(data(:,1)));
fprintf('  x position: [%.3f, %.3f] m\n', min(data(:,2)), max(data(:,2)));
fprintf('  y position: [%.3f, %.3f] m\n', min(data(:,3)), max(data(:,3)));
fprintf('  Joint angles: [%.3f, %.3f] rad\n', min(min(data(:,4:8))), max(max(data(:,4:8))));
fprintf('  Wheel angles: [%.3f, %.3f] rad\n', min(min(data(:,9:12))), max(max(data(:,9:12))));
fprintf('  Gripper state: [%.0f, %.0f]\n', min(data(:,13)), max(data(:,13)));

% Check gripper states
unique_gripper = unique(data(:,13));
if all(ismember(unique_gripper, [0, 1]))
    fprintf('✓ Gripper states are valid (0 or 1)\n');
else
    fprintf('✗ ERROR: Invalid gripper states found: %s\n', mat2str(unique_gripper'));
end

% Check for NaN or Inf
if any(isnan(data(:)))
    fprintf('✗ ERROR: File contains NaN values!\n');
elseif any(isinf(data(:)))
    fprintf('✗ ERROR: File contains Inf values!\n');
else
    fprintf('✓ No NaN or Inf values found\n');
end

% Check motion continuity (no huge jumps)
diffs = diff(data(:,1:12));
max_diff = max(abs(diffs(:)));
if max_diff > 1.0  % More than 1 radian or meter per timestep
    fprintf('⚠ WARNING: Large jumps detected (max: %.3f)\n', max_diff);
    [row, col] = find(abs(diffs) == max_diff, 1);
    fprintf('  At row %d, column %d\n', row, col);
else
    fprintf('✓ Motion appears continuous\n');
end

% Summary
fprintf('\n=====================================\n');
if cols == 13 && all(ismember(unique_gripper, [0, 1])) && ...
   ~any(isnan(data(:))) && ~any(isinf(data(:)))
    fprintf('✓ CSV FILE IS VALID for CoppeliaSim\n');
    
    % Estimate trajectory duration
    duration = rows * 0.01;  % 10ms per configuration
    fprintf('  Estimated duration: %.1f seconds\n', duration);
    
    % Check gripper operations
    gripper_changes = find(diff(data(:,13)) ~= 0);
    fprintf('  Gripper operations: %d\n', length(gripper_changes));
    if length(gripper_changes) >= 2
        fprintf('  First grasp at: %.1f s\n', gripper_changes(1)*0.01);
        fprintf('  Release at: %.1f s\n', gripper_changes(end)*0.01);
    end
else
    fprintf('✗ CSV FILE HAS ERRORS - Please fix before using\n');
end
fprintf('=====================================\n\n');

end