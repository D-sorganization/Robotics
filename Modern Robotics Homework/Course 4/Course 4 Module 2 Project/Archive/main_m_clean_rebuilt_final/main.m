clear; clc;

start = [-0.5, -0.5];
goal = [0.5, 0.5];
goal_threshold = 0.05;
max_nodes = 500;
step_size = 0.05;

% Load obstacles
obstacle_path = which('obstacles.csv');
obstacles = readmatrix(obstacle_path);

[path, nodes, path_indices] = RRT(start, goal, obstacles, goal_threshold, max_nodes, step_size);

% Save plot in same folder as main.m
script_dir = fileparts(mfilename('fullpath'));
plotRRT(obstacles, path, nodes, start, goal);
saveas(gcf, fullfile(script_dir, 'rrt_plot.png'));

% Save results
results_dir = fullfile(script_dir, '..', 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

% Write nodes.csv
fid = fopen(fullfile(results_dir, 'nodes.csv'), 'w');
fprintf(fid, '# nodes.csv file for V-REP kilobot motion planning scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Each line below has the form\n');
fprintf(fid, '# x,y\n');
fclose(fid);
dlmwrite(fullfile(results_dir, 'nodes.csv'), nodes(:,1:2), '-append');

% Write edges.csv
fid = fopen(fullfile(results_dir, 'edges.csv'), 'w');
fprintf(fid, '# edges.csv file for V-REP kilobot motion planning scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Each line below has the form\n');
fprintf(fid, '# start_index,end_index\n');
fclose(fid);
edge_indices = nodes(2:end, 3);
edges = [(edge_indices - 1), (2:size(nodes,1))' - 1];
dlmwrite(fullfile(results_dir, 'edges.csv'), edges, '-append');

% Write path.csv
fid = fopen(fullfile(results_dir, 'path.csv'), 'w');
fprintf(fid, '# path.csv file for V-REP kilobot motion planning scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Below is a solution path represented as a sequence of nodes of the graph.\n');
fprintf(fid, '# index_0,index_1,index_2,...\n');
fclose(fid);
if ~isempty(path_indices)
    dlmwrite(fullfile(results_dir, 'path.csv'), path_indices - 1, '-append');
else
    dlmwrite(fullfile(results_dir, 'path.csv'), [], '-append');
end

% Copy the obstacles.csv used
copyfile(obstacle_path, fullfile(results_dir, 'obstacles.csv'));

% Copy source code into code folder
code_dir = fullfile(script_dir, '..', 'code');
if ~exist(code_dir, 'dir')
    mkdir(code_dir);
end
main_path = fullfile(script_dir, 'main.m');
if exist(main_path, 'file') && ~strcmp(main_path, fullfile(code_dir, 'main.m'))
    copyfile(main_path, fullfile(code_dir, 'main.m'));
end
copyfile(which('RRT.m'), fullfile(code_dir, 'RRT.m'));
copyfile(which('plotRRT.m'), fullfile(code_dir, 'plotRRT.m'));
copyfile(which('collisionCheck.m'), fullfile(code_dir, 'collisionCheck.m'));
