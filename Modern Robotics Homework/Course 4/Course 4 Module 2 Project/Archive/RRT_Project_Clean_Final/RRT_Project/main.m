clear; clc;

start = [-0.5, -0.5];
goal = [0.5, 0.5];
goal_threshold = 0.05;
max_nodes = 500;
step_size = 0.05;

obstacles = readmatrix('obstacles.csv');
[path, nodes, path_indices] = RRT(start, goal, obstacles, goal_threshold, max_nodes, step_size);

% Plot and save figure
plotRRT(obstacles, path, nodes, start, goal);
saveas(gcf, fullfile('RRT_Project', 'rrt_plot.png'));

% Save results directly into RRT_Project/results
results_dir = fullfile('RRT_Project', 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

% Save nodes.csv
fid = fopen(fullfile(results_dir, 'nodes.csv'), 'w');
fprintf(fid, '# nodes.csv file for V-REP kilobot motion planning scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Each line below has the form\n');
fclose(fid);
dlmwrite(fullfile(results_dir, 'nodes.csv'), nodes(:,1:2), '-append');

% Save edges.csv
fid = fopen(fullfile(results_dir, 'edges.csv'), 'w');
fprintf(fid, '# edges.csv file for V-REP kilobot motion planning scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Each line below is of the form\n');
fclose(fid);
edge_indices = nodes(2:end, 3);
edges = [(edge_indices - 1), (2:size(nodes,1))' - 1];
dlmwrite(fullfile(results_dir, 'edges.csv'), edges, '-append');

% Save path.csv
fid = fopen(fullfile(results_dir, 'path.csv'), 'w');
fprintf(fid, '# path.csv file for V-REP kilobot motion planning scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Below is a solution path represented as a sequence of nodes of the graph.\n');
fclose(fid);
if ~isempty(path_indices)
    dlmwrite(fullfile(results_dir, 'path.csv'), path_indices - 1, '-append');
else
    dlmwrite(fullfile(results_dir, 'path.csv'), [], '-append');
end

% Copy original obstacles.csv into results folder
copyfile('obstacles.csv', fullfile(results_dir, 'obstacles.csv'));

% Copy code files into RRT_Project/code
code_dir = fullfile('RRT_Project', 'code');
if ~exist(code_dir, 'dir')
    mkdir(code_dir);
end
copyfile('main.m', fullfile(code_dir, 'main.m'));
copyfile('RRT.m', fullfile(code_dir, 'RRT.m'));
copyfile('plotRRT.m', fullfile(code_dir, 'plotRRT.m'));
copyfile('collisionCheck.m', fullfile(code_dir, 'collisionCheck.m'));
