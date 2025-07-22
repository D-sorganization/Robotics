function [path, nodes] = RRT(start, goal, obstacles, goal_thresh, max_nodes, step_size)

nodes = [start, 0]; % x, y, z, parent_index
path = [];

for i = 1:max_nodes
    if rand < 0.2
        sample = goal;
    else
        sample = rand(1, 3) - 0.5;  % uniform sample in [-0.5, 0.5]^3
    end

    % Find nearest node
    dists = vecnorm(nodes(:,1:3) - sample, 2, 2);
    [~, idx] = min(dists);
    nearest = nodes(idx,1:3);

    % Move toward sample
    direction = (sample - nearest) / norm(sample - nearest);
    new_node = nearest + step_size * direction;

    % Check collision
    if collisionCheck(new_node, nearest, obstacles)
        continue;
    end

    % Add new node
    nodes = [nodes; new_node, idx];

    % Check goal
    if norm(new_node - goal) < goal_thresh
        current = size(nodes,1);
        while current > 0
            path = [nodes(current,1:3); path];
            current = nodes(current,4);
        end
        disp('Goal reached!');
        return;
    end
end

disp('Goal not reached.');
end
