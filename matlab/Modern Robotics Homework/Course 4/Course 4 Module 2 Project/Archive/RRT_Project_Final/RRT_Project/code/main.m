clear; clc;

start = [-0.5, -0.5];
goal = [0.5, 0.5];
goal_threshold = 0.05;
max_nodes = 500;
step_size = 0.05;

obstacles = readmatrix('obstacles.csv');
[path, nodes] = RRT(start, goal, obstacles, goal_threshold, max_nodes, step_size);
plotRRT(obstacles, path, nodes, start, goal);

% Save outputs
writematrix(nodes, 'nodes.csv');
edges = nodes(2:end, [1 2 3]); % x, y, parent index
writematrix(edges, 'edges.csv');
writematrix(path, 'path.csv');
writematrix(obstacles, 'obstacles.csv');
