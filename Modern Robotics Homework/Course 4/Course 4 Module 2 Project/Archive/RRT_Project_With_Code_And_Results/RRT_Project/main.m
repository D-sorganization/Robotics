clear; clc;

start = [-0.5, -0.5];
goal = [0.5, 0.5];
goal_threshold = 0.05;
max_nodes = 500;
step_size = 0.05;

obstacles = readmatrix('obstacles.csv');
[path, nodes, path_indices] = RRT(start, goal, obstacles, goal_threshold, max_nodes, step_size);
plotRRT(obstacles, path, nodes, start, goal);

% Step 1: write results in root-level 'results'
root_results = 'results';
if ~exist(root_results, 'dir')
    mkdir(root_results);
end

% Write nodes.csv
fid = fopen(fullfile(root_results, 'nodes.csv'), 'w');
fprintf(fid, '# nodes.csv file for V-REP kilobot motion planning scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Each line below has the form\n');
fclose(fid);
dlmwrite(fullfile(root_results, 'nodes.csv'), nodes(:,1:2), '-append');

% Write edges.csv
fid = fopen(fullfile(root_results, 'edges.csv'), 'w');
fprintf(fid, '# edges.csv file for V-REP kilobot motion planning scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Each line below is of the form\n');
fclose(fid);
edge_indices = nodes(2:end, 3);
edges = [(edge_indices - 1), (2:size(nodes,1))' - 1];
dlmwrite(fullfile(root_results, 'edges.csv'), edges, '-append');

% Write path.csv
fid = fopen(fullfile(root_results, 'path.csv'), 'w');
fprintf(fid, '# path.csv file for V-REP kilobot motion planning scene.\n');
fprintf(fid, '# All lines beginning with a # are treated as a comment and ignored.\n');
fprintf(fid, '# Below is a solution path represented as a sequence of nodes of the graph.\n');
fclose(fid);
if ~isempty(path_indices)
    dlmwrite(fullfile(root_results, 'path.csv'), path_indices - 1, '-append');
else
    dlmwrite(fullfile(root_results, 'path.csv'), [], '-append');
end

% Save obstacles used
copyfile('obstacles.csv', fullfile(root_results, 'obstacles.csv'));

% Step 2: copy results into RRT_Project/results
destination = fullfile('RRT_Project', 'results');
if ~exist(destination, 'dir')
    mkdir(destination);
end
copyfile(fullfile(root_results, '*'), destination);


% Step 3: copy code files into RRT_Project/code
code_dir = fullfile('RRT_Project', 'code');
if ~exist(code_dir, 'dir')
    mkdir(code_dir);
end
copyfile('main.m', fullfile(code_dir, 'main.m'));
copyfile('RRT.m', fullfile(code_dir, 'RRT.m'));
copyfile('plotRRT.m', fullfile(code_dir, 'plotRRT.m'));
copyfile('collisionCheck.m', fullfile(code_dir, 'collisionCheck.m'));
