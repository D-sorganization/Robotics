clear; clc;

% Load map and assign to obstacles
load('map1.mat');     % Change to 'map2.mat', etc. as needed
obstacles = map;      % 'map' contains the [x, y, radius] for each obstacle

% Parameters
start = [0, 0];
goal = [1, 1];
stepSize = 0.05;
goalThreshold = 0.05;
maxIterations = 10000;

% Initialize RRT tree
nodes(1).pos = start;
nodes(1).parent = 0;

% Plot setup
figure; hold on; axis equal;
for i = 1:size(obstacles, 1)
    obs = obstacles(i,:);
    rectangle('Position', [obs(1)-obs(3), obs(2)-obs(3), 2*obs(3), 2*obs(3)], ...
              'Curvature', [1, 1], 'EdgeColor', 'k', 'LineWidth', 1.5);
end
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
xlim([0 1]); ylim([0 1]);

% Main RRT loop
for i = 1:maxIterations
    % 1. Sample a random point in [0,1] x [0,1]
    randPoint = rand(1, 2);
    
    % Skip if sampled point is in collision
    if checkCollision(randPoint, obstacles)
        continue;
    end

    % 2. Find nearest node
    allNodes = cell2mat(arrayfun(@(n)n.pos, nodes, 'UniformOutput', false))';
    dists = vecnorm(allNodes - randPoint, 2, 2);
    [~, idx] = min(dists);
    nearest = nodes(idx).pos;

    % 3. Steer towards the random point
    direction = randPoint - nearest;
    if norm(direction) > stepSize
        direction = direction / norm(direction) * stepSize;
    end
    newPoint = nearest + direction;

    % 4. Check for collisions
    if checkCollision(newPoint, obstacles) || checkEdgeCollision(nearest, newPoint, obstacles)
        continue;
    end

    % 5. Add new node
    newNode.pos = newPoint;
    newNode.parent = idx;
    nodes(end + 1) = newNode;

    % Draw the new edge
    line([nearest(1), newPoint(1)], [nearest(2), newPoint(2)], 'Color', 'b');

    % 6. Check for goal
    if norm(newPoint - goal) < goalThreshold
        disp('Goal reached!');
        % Add goal as final node
        nodes(end + 1).pos = goal;
        nodes(end).parent = length(nodes) - 1;
        break;
    end
end

% Reconstruct and plot final path
if norm(nodes(end).pos - goal) < goalThreshold
    current = length(nodes);
    while nodes(current).parent ~= 0
        parent = nodes(current).parent;
        line([nodes(current).pos(1), nodes(parent).pos(1)], ...
             [nodes(current).pos(2), nodes(parent).pos(2)], ...
             'Color', 'r', 'LineWidth', 2);
        current = parent;
    end
    title('RRT Path Found');
else
    title('Failed to Reach Goal');
end

% --- Collision Functions ---

function inCollision = checkCollision(point, obstacles)
    inCollision = false;
    for i = 1:size(obstacles,1)
        obs = obstacles(i,:);
        if norm(point - obs(1:2)) <= obs(3)
            inCollision = true;
            return;
        end
    end
end

function inCollision = checkEdgeCollision(p1, p2, obstacles)
    numSteps = 50;
    inCollision = false;
    for i = 0:numSteps
        t = i / numSteps;
        pt = p1 + t * (p2 - p1);
        if checkCollision(pt, obstacles)
            inCollision = true;
            return;
        end
    end
end
