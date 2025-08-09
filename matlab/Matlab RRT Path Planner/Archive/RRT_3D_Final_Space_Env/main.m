clear; clc;

obstacles_raw = readmatrix('obstacles3D_oblong_colored.csv');

bounds = obstacles_raw(1,2:7);  % x_min, x_max, y_min, y_max, z_min, z_max
obstacles = obstacles_raw(2:end,:);  % actual obstacles data

start = [bounds(1), bounds(3), bounds(5)];
goal = [bounds(2), bounds(4), bounds(6)];

goal_threshold = 0.1;
max_nodes = 2000;
step_size = 0.05;

[path, nodes] = RRT(start, goal, obstacles, bounds, goal_threshold, max_nodes, step_size);

plotRRT(obstacles, path, nodes, start, goal, bounds);
