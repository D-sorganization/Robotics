clear; clc;

start = [-0.5, -0.5, -0.5];
goal = [0.5, 0.5, 0.5];
goal_threshold = 0.05;
max_nodes = 5000;
step_size = 0.05;

obstacles = readmatrix('obstacles3D.csv');

[path, nodes] = RRT(start, goal, obstacles, goal_threshold, max_nodes, step_size);

% Plot the RRT and the final path
plotRRT(obstacles, path, nodes, start, goal);
