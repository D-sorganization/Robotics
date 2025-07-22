%% Mobile Manipulation Capstone Project
% Northwestern University - Modern Robotics
% 
% This script runs the complete mobile manipulation simulation
% for the KUKA youBot picking and placing a block.

clear; clc; close all;

fprintf('================================================\n');
fprintf('  Mobile Manipulation Capstone Project\n');
fprintf('  KUKA youBot Pick and Place Simulation\n');
fprintf('================================================\n\n');

%% Select simulation mode
fprintf('Select simulation mode:\n');
fprintf('1. Run all tests (milestones)\n');
fprintf('2. Run best controller\n');
fprintf('3. Run overshoot controller\n');
fprintf('4. Run new task\n');
fprintf('5. Run all three final simulations\n');
fprintf('6. Run complete project (like reference)\n');
fprintf('7. Exit\n\n');

choice = input('Enter your choice (1-7): ');

switch choice
    case 1
        fprintf('\n=== Running Milestone Tests ===\n\n');
        % Run the test script directly
        runTests;
        
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
        fprintf('\n=== Running All Three Simulations ===\n\n');
        
        % Run best controller
        fprintf('1/3: Best Controller\n');
        runSimulation('best');
        pause(2);
        
        % Run overshoot controller
        fprintf('\n2/3: Overshoot Controller\n');
        runSimulation('overshoot');
        pause(2);
        
        % Run new task
        fprintf('\n3/3: New Task\n');
        runSimulation('newTask');
        
        fprintf('\n=== All simulations complete! ===\n');
        fprintf('Check the results/ directory for outputs\n');
        
    case 6
        fprintf('\n=== Running Complete Project ===\n');
        runProject;
        
    case 7
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