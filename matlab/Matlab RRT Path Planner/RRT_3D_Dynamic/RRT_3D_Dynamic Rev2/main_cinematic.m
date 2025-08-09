% main_cinematic.m (Version 3 - Corrected)

clc;
clear;
close all;

disp('--- Running Cinematic Falcon Planner ---');

% --- Centralized Configuration ---
% Grouping parameters here makes them easy to change
config.max_nodes = 4000;
config.step_size = 0.06;
config.goal_radius = 0.12;
config.goal_bias = 0.2; % 20% chance to aim directly at goal

% --- Setup Scene ---
% This is the only place where these variables should be defined.
obstacles = readmatrix('obstacles3D.csv');
bounds = [-1.0, 1.0, -0.6, 0.6, -0.3, 0.3];
start = [-0.9, 0, 0];
goal = [0.9, 0, 0];

% --- Run RRT with Config ---
% We pass all necessary variables as arguments.
disp('Running RRT Planner...');
[nodes, path] = RRT(start, goal, obstacles, bounds, config);

if isempty(path)
    error('❌ No valid path found.');
end

% --- Create Animation ---
% The 'plotRRT_Moving' function receives the variables it needs.
plotRRT_Moving(obstacles, path, nodes, start, goal, bounds);

disp('✅ Cinematic video generation complete.');