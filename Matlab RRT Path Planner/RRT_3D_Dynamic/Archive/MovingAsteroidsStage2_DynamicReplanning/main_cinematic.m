clear; clc;
obstacles_raw = readmatrix('obstacles3D.csv');
bounds = obstacles_raw(1,2:7);
obstacles = obstacles_raw(2:end,:);

start = [bounds(1), bounds(3), bounds(5)];
goal = [bounds(2), bounds(4), bounds(6)];

goal_threshold = 0.1;
max_nodes = 2000;
step_size = 0.05;

[path, nodes] = RRT(start, goal, obstacles, bounds, goal_threshold, max_nodes, step_size);

% Plot moving asteroids and Falcon flying
plotRRT_Moving(obstacles, path, nodes, start, goal, bounds);
