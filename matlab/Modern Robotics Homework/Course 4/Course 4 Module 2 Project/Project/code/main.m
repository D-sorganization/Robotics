clear; clc;

start = [-0.5, -0.5];
goal = [0.5, 0.5];
goal_threshold = 0.05;
max_nodes = 500;
step_size = 0.05;

obstacle_path = which('obstacles.csv');
obstacles = readmatrix(obstacle_path);

[path, nodes, path_indices] = RRT(start, goal, obstacles, goal_threshold, max_nodes, step_size);

script_dir = fileparts(mfilename('fullpath'));
plotRRT(obstacles, path, nodes, start, goal);
saveas(gcf, fullfile(script_dir, 'rrt_plot.png'));

results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

% Write nodes.csv with ID, x, y, heuristic_cost
fid = fopen(fullfile(results_dir, 'nodes.csv'), 'w');
if fid == -1
    error('Failed to open nodes.csv for writing.');
end
fprintf(fid, '# nodes.csv file for A* graph search scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Each line below has the form\n');
fprintf(fid, '# id,x,y,heuristic_cost\n');
fclose(fid);
heuristics = vecnorm(nodes(:,1:2) - goal, 2, 2);
node_ids = (1:size(nodes,1))';
nodes_with_heuristic = [node_ids, nodes(:,1:2), heuristics];
dlmwrite(fullfile(results_dir, 'nodes.csv'), nodes_with_heuristic, '-append');

% Write edges.csv sorted by the order RRT added them (by end_id, ascending)
fid = fopen(fullfile(results_dir, 'edges.csv'), 'w');
if fid == -1
    error('Failed to open edges.csv for writing.');
end
fprintf(fid, '# edges.csv file for A* graph search scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Each line below has the form\n');
fprintf(fid, '# id1,id2,cost\n');
fclose(fid);

start_ids = nodes(2:end, 3);          % parent (1-based)
end_ids = (2:size(nodes,1))';         % child (1-based)
costs = vecnorm(nodes(2:end,1:2) - nodes(start_ids,1:2), 2, 2);
edges = [start_ids, end_ids, costs];

dlmwrite(fullfile(results_dir, 'edges.csv'), edges, '-append');

% Write path.csv with node IDs
fid = fopen(fullfile(results_dir, 'path.csv'), 'w');
if fid == -1
    error('Failed to open path.csv for writing.');
end
fprintf(fid, '# path.csv file for A* graph search scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Below is a solution path represented as a sequence of node IDs.\n');
fprintf(fid, '# id_1,id_2,id_3,...\n');
fclose(fid);
if ~isempty(path_indices)
    dlmwrite(fullfile(results_dir, 'path.csv'), path_indices, '-append');
else
    dlmwrite(fullfile(results_dir, 'path.csv'), [], '-append');
end

% % Copy obstacles file
% copyfile(obstacle_path, fullfile(results_dir, 'obstacles.csv'));
