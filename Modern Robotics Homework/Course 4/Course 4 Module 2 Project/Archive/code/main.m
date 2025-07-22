clear; clc;

start = [-0.5, -0.5];
goal = [0.5, 0.5];
goal_threshold = 0.05;
max_nodes = 500;
step_size = 0.05;

obstacles = readmatrix('obstacles.csv');
[path, nodes, path_indices] = RRT(start, goal, obstacles, goal_threshold, max_nodes, step_size);
plotRRT(obstacles, path, nodes, start, goal);

% Save outputs
mkdir results\;
cd results\;
writematrix(nodes, 'nodes.csv');
edges = nodes(2:end, [1 2 3]); % x, y, parent index
writematrix(edges, 'edges.csv');
if ~isempty(path_indices)
    writematrix(path_indices, 'path.csv'); % node indices in row vector
else
    writematrix([], 'path.csv'); % empty if no path found
end
writematrix(obstacles, 'obstacles.csv');
