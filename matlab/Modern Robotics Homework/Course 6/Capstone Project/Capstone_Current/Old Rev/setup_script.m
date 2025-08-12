% setup.m - Setup script for Mobile Manipulation Capstone Project
% Run this script once to set up the MATLAB environment

fprintf('Setting up Mobile Manipulation Capstone Project...\n');

% Add current directory to path
addpath(pwd);

% Check for Modern Robotics library
mr_functions = {'FKinBody', 'FKinSpace', 'JacobianBody', 'JacobianSpace', ...
                'MatrixExp6', 'MatrixLog6', 'VecTose3', 'se3ToVec', ...
                'Adjoint', 'TransInv', 'AxisAng3', 'VecToso3'};

fprintf('\nChecking for Modern Robotics library...\n');
mr_found = true;
for i = 1:length(mr_functions)
    if ~exist(mr_functions{i}, 'file')
        fprintf('  ✗ %s not found\n', mr_functions{i});
        mr_found = false;
    end
end

if mr_found
    fprintf('✓ Modern Robotics library found!\n');
else
    fprintf('\n⚠ Modern Robotics library not found!\n');
    fprintf('Please download it from:\n');
    fprintf('https://github.com/NxRLab/ModernRobotics\n');
    fprintf('And add the MATLAB package to your path:\n');
    fprintf('addpath(''path/to/ModernRobotics/packages/MATLAB/mr'')\n\n');
    return;
end

% Check for required project files
fprintf('\nChecking for project files...\n');
required_files = {
    % Core functions
    'NextState.m',
    'TrajectoryGenerator.m', 
    'FeedbackControl.m',
    'youBotKinematics.m',
    'runSimulation.m',
    % Utility functions
    'writeCSV.m',
    'plotError.m',
    'checkJointLimits.m',
    'applyJointLimits.m',
    % Helper functions (project-specific)
    'RpToTrans.m',
    'RotZ.m',
    % Scripts
    'main.m',
    'runTests.m',
    'runProject.m',
    % Optional but useful
    'verifyCSV.m'
};

missing_files = {};
for i = 1:length(required_files)
    if ~exist(required_files{i}, 'file')
        missing_files{end+1} = required_files{i};
    end
end

if ~isempty(missing_files)
    fprintf('\n⚠ Warning: The following project files are missing:\n');
    for i = 1:length(missing_files)
        fprintf('  - %s\n', missing_files{i});
    end
    fprintf('\nPlease ensure all project files are in the current directory.\n');
else
    fprintf('✓ All required project files found!\n');
end

% Create results directory if it doesn't exist
if ~exist('results', 'dir')
    mkdir('results');
    fprintf('\nCreated results/ directory\n');
end

fprintf('\n================================================\n');
fprintf('Setup complete. Run "main" to start the simulation.\n');
fprintf('================================================\n');