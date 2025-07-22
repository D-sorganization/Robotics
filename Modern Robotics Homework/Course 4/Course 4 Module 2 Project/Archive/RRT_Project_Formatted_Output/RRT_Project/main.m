clear; clc;

start = [-0.5, -0.5];
goal = [0.5, 0.5];
goal_threshold = 0.05;
max_nodes = 500;
step_size = 0.05;

obstacles = readmatrix('obstacles.csv');
[path, nodes, path_indices] = RRT(start, goal, obstacles, goal_threshold, max_nodes, step_size);
plotRRT(obstacles, path, nodes, start, goal);

% Write nodes.csv
fid = fopen('nodes.csv', 'w');
fprintf(fid, '# nodes.csv file for V-REP kilobot motion planning scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Each line below has the form\n');
fclose(fid);
dlmwrite('nodes.csv', nodes(:,1:2), '-append');

% Write edges.csv
fid = fopen('edges.csv', 'w');
fprintf(fid, '# edges.csv file for V-REP kilobot motion planning scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Each line below is of the form\n');
fclose(fid);
edges = nodes(2:end, 3);
edge_data = [(edges - 1), (2:size(nodes,1))' - 1];  % zero-based indexing
dlmwrite('edges.csv', edge_data, '-append');

% Write path.csv
fid = fopen('path.csv', 'w');
fprintf(fid, '# path.csv file for V-REP kilobot motion planning scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Below is a solution path represented as a sequence of nodes of the graph.\n');
fclose(fid);
if ~isempty(path_indices)
    dlmwrite('path.csv', path_indices - 1, '-append');  % zero-based indexing
else
    dlmwrite('path.csv', [], '-append');
end

% Also save obstacles for reference
writematrix(obstacles, 'obstacles_out.csv');
