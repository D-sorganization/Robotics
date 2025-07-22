function [nodes, path] = RRT(start, goal, obstacles, bounds, config)
% RRT.m (Version 2 - Cleaned)
% Builds a Rapidly-Exploring Random Tree (RRT) with goal biasing.
% This version is optimized for performance with pre-allocation.
%
% Outputs:
%   nodes: [x y z parent_index]
%   path:  final path from start to goal (Nx3)

% --- Configuration ---
max_nodes   = config.max_nodes;
step_size   = config.step_size;
goal_radius = config.goal_radius;
goal_bias   = config.goal_bias;

% --- Robustness Check ---
% Ensure start/goal are not inside an obstacle before beginning
if collisionCheck(start, obstacles)
    error('❌ Start point is inside an obstacle.');
end
if collisionCheck(goal, obstacles)
    error('❌ Goal point is inside an obstacle.');
end

% --- Performance: Pre-allocate nodes array ---
nodes = zeros(max_nodes, 4); % [x, y, z, parent_index]
nodes(1, :) = [start, 0];    % Add the start node
node_count = 1;              % Keep track of current number of nodes

path = []; % Initialize path as empty

for iter = 1:max_nodes
    % --- Goal-biased random sampling ---
    if rand() < goal_bias
        rand_point = goal;
    else
        rand_point = [
            (bounds(2)-bounds(1))*rand() + bounds(1), ...
            (bounds(4)-bounds(3))*rand() + bounds(3), ...
            (bounds(6)-bounds(5))*rand() + bounds(5)  ...
        ];
    end
    rand_point = rand_point(:)'; % Force row vector

    % --- Find nearest node ---
    % This is faster because `nodes` is not growing dynamically
    diffs = nodes(1:node_count, 1:3) - rand_point;
    dists = vecnorm(diffs, 2, 2);
    [~, nearest_idx] = min(dists);
    nearest_node = nodes(nearest_idx, 1:3);

    % --- Step towards the random point ---
    direction = rand_point - nearest_node;
    d = norm(direction);
    if d < 1e-6; continue; end % Skip if direction is negligible
    direction = direction / d;
    new_node_pos = nearest_node + step_size * direction;

    % --- Check for collision ---
    if collisionCheck(new_node_pos, obstacles)
        continue;
    end

    % --- Add new node (Efficiently) ---
    node_count = node_count + 1;
    nodes(node_count, :) = [new_node_pos, nearest_idx];

    % --- Check if goal is reached ---
    if norm(new_node_pos - goal) < goal_radius
        disp('✅ Goal Reached!');
        
        % Backtrack to find the path
        path = new_node_pos;
        curr_idx = node_count;
        while curr_idx ~= 0
            parent_idx = nodes(curr_idx, 4);
            if parent_idx == 0; break; end
            path = [nodes(parent_idx, 1:3); path];
            curr_idx = parent_idx;
        end
        break; % Exit loop
    end
end

% --- Trim unused part of the nodes array ---
nodes = nodes(1:node_count, :);

if isempty(path)
    disp('⚠️ Goal not reached within max_nodes.');
end

end