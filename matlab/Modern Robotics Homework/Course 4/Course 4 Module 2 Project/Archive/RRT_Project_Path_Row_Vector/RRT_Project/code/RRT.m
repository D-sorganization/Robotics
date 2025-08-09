function [path, nodes] = RRT(start, goal, obstacles, goal_thresh, max_nodes, step_size)

nodes = [start, 0];
path = [];

for i = 1:max_nodes
    if rand < 0.2
        sample = goal;
    else
        sample = rand(1, 2) - 0.5;  % sample from [-0.5, 0.5] x [-0.5, 0.5]
    end

    [~, idx] = min(vecnorm(nodes(:,1:2) - sample, 2, 2));
    nearest = nodes(idx, 1:2);

    direction = sample - nearest;
    direction = direction / norm(direction);
    new_node = nearest + step_size * direction;

    if collisionCheck(nearest, new_node, obstacles)
        continue;
    end

    nodes = [nodes; new_node, idx];

    if norm(new_node - goal) < goal_thresh
        current = size(nodes,1);
        while current > 0
            path = [nodes(current,1:2); path];
            current = nodes(current,3);
        end
        disp('Goal reached!');
        return;
    end
end

disp('Goal not reached within node limit.');
end