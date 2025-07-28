%% Mobile Manipulation Capstone Project
% Northwestern University - Modern Robotics
% 
% This script runs the complete mobile manipulation simulation
% for the KUKA youBot picking and placing a block.

function run_capstone_project()
% Main script function to encapsulate workspace and enable cleanup

clear; clc; close all;

fprintf('================================================\n');
fprintf('  Mobile Manipulation Capstone Project\n');
fprintf('  KUKA youBot Pick and Place Simulation\n');
fprintf('================================================\n\n');

%% Add required paths
current_dir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(current_dir, 'Functions')));

%% Select simulation mode
fprintf('Select simulation mode:\n');
fprintf('1. Run all tests (milestones)\n');
fprintf('2. Run best controller (working project match)\n');
fprintf('3. Run overshoot controller\n');
fprintf('4. Run new task\n');
fprintf('5. Run optimal controller (tuned gains)\n');
fprintf('6. Run robust controller (conservative)\n');
fprintf('7. Run all final simulations\n');
fprintf('8. Run complete project (like reference)\n');
fprintf('9. Exit\n\n');

choice = input('Enter your choice (1-9): ');

% Store initial directory and files for cleanup
initial_dir = pwd;
initial_files = dir('*.csv');
initial_mat_files = dir('*.mat');
initial_pdf_files = dir('*.pdf');

try
    switch choice
        case 1
            fprintf('\n=== Running Milestone Tests ===\n\n');
            % Run the test script directly
            if exist('runTests.m', 'file')
                runTests;
            else
                fprintf('Warning: runTests.m not found. Skipping milestone tests.\n');
            end
            
        case 2
            fprintf('\n=== Running Best Controller ===\n');
            runSimulation('best');
            
        case 3
            fprintf('\n=== Running Overshoot Controller ===\n');
            runSimulation('overshoot');
            
        case 4
            fprintf('\n=== Running New Task ===\n');
            runSimulation('newTask');
            
        case 5
            fprintf('\n=== Running Optimal Controller ===\n');
            runSimulation('optimal');
            
        case 6
            fprintf('\n=== Running Robust Controller ===\n');
            runSimulation('robust');
            
        case 7
            fprintf('\n=== Running All Final Simulations ===\n\n');
            
            % Run best controller
            fprintf('1/5: Best Controller (Working Project Match)\n');
            runSimulation('best');
            pause(2);
            
            % Run overshoot controller
            fprintf('\n2/5: Overshoot Controller\n');
            runSimulation('overshoot');
            pause(2);
            
            % Run new task
            fprintf('\n3/5: New Task\n');
            runSimulation('newTask');
            pause(2);
            
            % Run optimal controller
            fprintf('\n4/5: Optimal Controller\n');
            runSimulation('optimal');
            pause(2);
            
            % Run robust controller
            fprintf('\n5/5: Robust Controller\n');
            runSimulation('robust');
            
            fprintf('\n=== All simulations complete! ===\n');
            fprintf('Check the results/ directory for outputs\n');
            
        case 8
            fprintf('\n=== Running Complete Project ===\n');
            if exist('runProject.m', 'file')
                runProject;
            else
                fprintf('Warning: runProject.m not found. Running best controller instead.\n');
                runSimulation('best');
            end
            
        case 9
            fprintf('\nExiting...\n');
            return;
            
        otherwise
            fprintf('\nInvalid choice. Please run the script again.\n');
            return;
    end
    
    fprintf('\n================================================\n');
    fprintf('  Simulation Complete!\n');
    fprintf('================================================\n\n');
    
    % Display instructions for CoppeliaSim
    fprintf('To visualize the results in CoppeliaSim:\n');
    fprintf('1. Open CoppeliaSim\n');
    fprintf('2. Load Scene 6: CSV Mobile Manipulation youBot\n');
    fprintf('3. Click "Load CSV" and select a trajectory.csv file from results/\n');
    fprintf('4. Click "Play File" to watch the simulation\n\n');
    
    fprintf('For end-effector trajectory visualization:\n');
    fprintf('1. Load Scene 8: CSV youBot End-Effector Animation\n');
    fprintf('2. Load test_milestone2_ee_trajectory.csv\n');
    fprintf('3. Click "Play File" to see the reference trajectory\n\n');
    
catch ME
    fprintf('\nError occurred during simulation:\n');
    fprintf('Error: %s\n', ME.message);
    fprintf('File: %s (line %d)\n', ME.stack(1).file, ME.stack(1).line);
end

%% Cleanup workspace and temporary files
cleanup_workspace(initial_files, initial_mat_files, initial_pdf_files);

end

function cleanup_workspace(initial_files, initial_mat_files, initial_pdf_files)
% Clean up temporary files and workspace
fprintf('\n=== Cleaning up workspace ===\n');

% Get current files
current_files = dir('*.csv');
current_mat_files = dir('*.mat');
current_pdf_files = dir('*.pdf');

% Find temporary CSV files (not in results directory)
temp_csv_files = {};
for i = 1:length(current_files)
    filename = current_files(i).name;
    % Check if this is a new file (not in initial list)
    is_new = true;
    for j = 1:length(initial_files)
        if strcmp(filename, initial_files(j).name)
            is_new = false;
            break;
        end
    end
    
    % Mark trajectory files as temporary (these should be in results/)
    if is_new || contains(filename, 'Traj.csv') || contains(filename, '_Traj.csv')
        temp_csv_files{end+1} = filename;
    end
end

% Find temporary MAT files
temp_mat_files = {};
for i = 1:length(current_mat_files)
    filename = current_mat_files(i).name;
    is_new = true;
    for j = 1:length(initial_mat_files)
        if strcmp(filename, initial_mat_files(j).name)
            is_new = false;
            break;
        end
    end
    if is_new && ~strcmp(filename, 'Xerr.mat')  % Keep Xerr.mat if in results
        temp_mat_files{end+1} = filename;
    end
end

% Find temporary PDF files
temp_pdf_files = {};
for i = 1:length(current_pdf_files)
    filename = current_pdf_files(i).name;
    is_new = true;
    for j = 1:length(initial_pdf_files)
        if strcmp(filename, initial_pdf_files(j).name)
            is_new = false;
            break;
        end
    end
    if is_new && strcmp(filename, 'error_plot.pdf')  % Only remove error_plot.pdf
        temp_pdf_files{end+1} = filename;
    end
end

% Remove temporary files
files_removed = 0;
all_temp_files = [temp_csv_files, temp_mat_files, temp_pdf_files];

for i = 1:length(all_temp_files)
    filename = all_temp_files{i};
    if exist(filename, 'file')
        try
            delete(filename);
            fprintf('  Removed: %s\n', filename);
            files_removed = files_removed + 1;
        catch
            fprintf('  Warning: Could not remove %s\n', filename);
        end
    end
end

% Clear workspace variables (except function workspace)
evalin('base', 'clear');
evalin('base', 'clc');

% Close any open figures (except if user wants to keep them)
close_figures = true;
if length(findall(0,'Type','figure')) > 0
    response = input('Close all figures? (y/n) [y]: ', 's');
    if isempty(response)
        response = 'y';
    end
    close_figures = strcmpi(response, 'y');
end

if close_figures
    close all;
    fprintf('  Closed all figures\n');
end

if files_removed > 0
    fprintf('  Total files cleaned up: %d\n', files_removed);
else
    fprintf('  No temporary files to clean up\n');
end

fprintf('  Workspace variables cleared\n');
fprintf('=== Cleanup complete ===\n\n');

% Display final status
if exist('results', 'dir')
    fprintf('Results preserved in results/ directory:\n');
    result_dirs = dir('results');
    for i = 1:length(result_dirs)
        if result_dirs(i).isdir && ~strcmp(result_dirs(i).name, '.') && ~strcmp(result_dirs(i).name, '..')
            fprintf('  - results/%s/\n', result_dirs(i).name);
        end
    end
else
    fprintf('No results directory found\n');
end

end

% Run the main script if called directly
if ~exist('skip_main_execution', 'var')
    run_capstone_project();
end