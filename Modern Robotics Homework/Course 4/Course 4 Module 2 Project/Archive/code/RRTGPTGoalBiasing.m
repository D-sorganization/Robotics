clear; clc;

% Load obstacles from CSV file (format: x, y, radius)
obstacles = csvread('obstacles.csv');

% Parameters
start = [-0.5, -0.5];
goal = [0.5, 0.5];
stepSize = 0.03;
goalThreshold = 0.05;
maxIterations = 20000;
goalBias = 0.1;

% Initialize RRT
nodes(1).pos = start;
nodes(1).parent = 0;

% Plot setup
figure; hold on; axis equal;
xlim([-0.5, 0.5]); ylim([-0.5, 0.5]);
for i = 1:size(obstacles, 1)
    obs = obstacles(i,:);
    rectangle('Position', [obs(1)-obs(3), obs(2)-obs(3), 2*obs(3), 2*obs(3)], ...
              'Curvature', [1, 1], 'EdgeColor', 'k', 'LineWidth', 1.5);
end
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% RRT Main Loop
for i = 1:maxIterations
    % Sample random point (with goal bias)
    if rand < goalBias
        randPoint = goal;
    else
        randPoint = -0.5 + rand(1, 2);  % Sample in [-0.5, 0.5]
    end

    if checkCollision(randPoint, obstacles)
        continue;
    end

    allNodes = cell2mat(arrayfun(@(n)n.pos, nodes, 'UniformOutput', false))';
    dists = vecnorm(allNodes - randPoint, 2, 2);
    [~, idx] = min(dists);
    nearest = nodes(idx).pos;

    direction = randPoint - nearest;
    if norm(direction) > stepSize
        direction = direction / norm(direction) * stepSize;
    end
    newPoint = nearest + direction;

    if checkCollision(newPoint, obstacles) || checkEdgeCollision(nearest, newPoint, obstacles)
        continue;
    end

    newNode.pos = newPoint;
    newNode.parent = idx;
    nodes(end + 1) = newNode;

    line([nearest(1), newPoint(1)], [nearest(2), newPoint(2)], 'Color', 'b');

    if norm(newPoint - goal) < goalThreshold && ...
       ~checkEdgeCollision(newPoint, goal, obstacles)
        disp('Goal reached!');
        goalNode.pos = goal;
        goalNode.parent = length(nodes);
        nodes(end + 1) = goalNode;
        line([newPoint(1), goal(1)], [newPoint(2), goal(2)], 'Color', 'g', 'LineStyle', '--');
        break;
    end
end

% Reconstruct and plot path (or closest attempt)
goalReached = norm(nodes(end).pos - goal) < goalThreshold;

if goalReached
    current = length(nodes);
    titleText = 'RRT Path Found';
else
    allNodes = cell2mat(arrayfun(@(n)n.pos, nodes, 'UniformOutput', false))';
    dists = vecnorm(allNodes - goal, 2, 2);
    [~, current] = min(dists);
    line([nodes(current).pos(1), goal(1)], [nodes(current).pos(2), goal(2)], ...
         'Color', 'm', 'LineStyle', '--', 'LineWidth', 1.5);
    titleText = 'Closest Attempt (Goal Not Reached)';
end

while nodes(current).parent ~= 0
    parent = nodes(current).parent;
    line([nodes(current).pos(1), nodes(parent).pos(1)], ...
         [nodes(current).pos(2), nodes(parent).pos(2)], ...
         'Color', 'r', 'LineWidth', 2);
    current = parent;
end

title(titleText);
disp(['Tree size: ', num2str(length(nodes))]);

% --- Collision Checking Functions ---
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
