% rrt_assignment.m
% This script implements a simple RRT algorithm for 2D path planning.
% It generates an RRT that expands from a given start point toward a goal point.
% When the algorithm finds a node that is within a goal threshold, it backtracks 
% to extract the final path. All computed data are saved in CSV files within a
% folder called "results".
%
% The following CSV files are created:
%   - nodes.csv: Each row is [NodeID, x-coordinate, y-coordinate, ParentNodeID].
%   - edges.csv: Each row is [ChildNodeID, ParentNodeID].
%   - path.csv: Contains the final path from start to goal (if found).
%
% To run:
% 1. Place this script in your MATLAB working directory.
% 2. Run this script by typing "rrt_assignment" on the command line or pressing Run.
% 3. Explore the "results" folder for the CSV outputs.
%
% Author: Dieter Olson
% Date: April 2025

clear 
clc;

%% Parameters and Workspace Setup

% Define workspace boundaries (a 100 x 100 square)
x_limit = [0, 100];  % x boundaries
y_limit = [0, 100];  % y boundaries

% Define start and goal positions [x, y]
start = [10, 10];
goal  = [90, 90];

% RRT parameters
max_iterations = 5000;    % Maximum number of iterations
step_size = 5;            % Distance to extend from nearest node
goal_threshold = 5;       % Radius within which the goal is considered reached

rng('shuffle');  % Seed the random number generator for variability

%% Initialize the RRT Tree
% Each node is stored as: [NodeID, x, y, ParentNodeID]
tree.nodes = [];
tree.nodes(1,:) = [1, start, -1];  % Root node (start) has no parent (-1)
node_count = 1;
goal_reached = false;
goal_node_id = -1;

%% Begin RRT Algorithm
for iter = 1:max_iterations
    % Sample a random point in the 2D workspace
    x_rand = [rand() * diff(x_limit) + x_limit(1), rand() * diff(y_limit) + y_limit(1)];
    
    % Find the nearest node in the current tree (Euclidean distance)
    distances = sqrt((tree.nodes(:,2) - x_rand(1)).^2 + (tree.nodes(:,3) - x_rand(2)).^2);
    [~, idx] = min(distances);
    nearest_node = tree.nodes(idx, 2:3);
    
    % Compute a new node in the direction of the random sample
    theta = atan2(x_rand(2) - nearest_node(2), x_rand(1) - nearest_node(1));
    new_x = nearest_node(1) + step_size * cos(theta);
    new_y = nearest_node(2) + step_size * sin(theta);
    
    % Ensure the new node is within the workspace boundaries
    if new_x < x_limit(1) || new_x > x_limit(2) || new_y < y_limit(1) || new_y > y_limit(2)
        continue;  % Skip if the node is out of bounds
    end
    
    % Add the new node to the tree
    node_count = node_count + 1;
    tree.nodes(node_count,:) = [node_count, new_x, new_y, tree.nodes(idx,1)];
    
    % Check if the new node is close enough to the goal
    if sqrt((new_x - goal(1))^2 + (new_y - goal(2))^2) < goal_threshold
        goal_reached = true;
        goal_node_id = node_count;
        fprintf('Goal reached at iteration %d\n', iter);
        break;
    end
end

%% Backtrack to Build the Final Path
path = [];
if goal_reached
    current = goal_node_id;
    while current ~= -1
        % Prepend the node's coordinates to the path
        path = [tree.nodes(current, 2:3); path];
        current = tree.nodes(current, 4);  % move to parent node
    end
else
    fprintf('Goal not reached within maximum iterations.\n');
end

%% Save CSV Files in "results" Folder
resultsFolder = 'results';
if ~exist(resultsFolder, 'dir')
    mkdir(resultsFolder);
end

% Save nodes.csv: [NodeID, x, y, ParentNodeID]
nodes_file = fullfile(resultsFolder, 'nodes.csv');
csvwrite(nodes_file, tree.nodes);

% Save edges.csv: [ChildNodeID, ParentNodeID]
edges = tree.nodes(:, [1, 4]);
edges_file = fullfile(resultsFolder, 'edges.csv');
csvwrite(edges_file, edges);

% Save path.csv if a goal was reached
if goal_reached
    path_file = fullfile(resultsFolder, 'path.csv');
    csvwrite(path_file, path);
end

fprintf('Results have been saved in the folder: %s\n', resultsFolder);