clear; clc;

% --- Load obstacles (skip comments and blanks) ---
rawLines = readlines("obstacles.csv");
obstacles = [];

for i = 1:length(rawLines)
    line = strtrim(rawLines(i));
    if startsWith(line, "#") || line == ""
        continue;
    end
    nums = sscanf(line, '%f,%f,%f');
    if numel(nums) == 3
        obstacles(end+1, :) = nums'; %#ok<SAGROW>
    end
end

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
    if all(isfinite(obs)) && numel(obs) == 3
        rectangle('Position', [obs(1)-obs(3), obs(2)-obs(3), 2*obs(3), 2*obs(3)], ...
                  'Curvature', [1, 1], 'EdgeColor', 'k', 'LineWidth', 1.5);
    end
end
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% --- RRT Main Loop ---
for i = 1:maxIterations
    if rand < goalBias
        randPoint = goal;
    else
        randPoint = -0.5 + rand(1, 2);  % in [-0.5, 0.5]
    end

    if checkCollision(randPoint, obstacles)
        continue;
    end

    try
        allNodes = vertcat(nodes.pos);
        dists = vecnorm(allNodes - randPoint, 2, 2);
        [~, idx] = min(dists);
        nearest = nodes(idx).pos;
    catch
        warning('Skipping iteration: nearest node not found.');
        continue;
    end

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

    if all(size(nearest) == [1, 2]) && all(size(newPoint) == [1, 2])
        line([nearest(1), newPoint(1)], [nearest(2), newPoint(2)], 'Color', 'b');
    else
        warning('Invalid point(s) for plotting edge.');
    end

    if norm(newPoint - goal) < goalThreshold && ...
       ~checkEdgeCollision(newPoint, goal, obstacles)
        disp('Goal reached!');
        goalNode.pos = goal;
        goalNode.parent = length(nodes);
        nodes(end + 1) = goalNode;

        % Safely connect last valid node to goal
        parentNode = nodes(goalNode.parent).pos;
        line([parentNode(1), goal(1)], [parentNode(2), goal(2)], 'Color', 'g', 'LineStyle', '--');
        break;
    end
end

% --- Path reconstruction ---
goalReached = norm(nodes(end).pos - goal) < goalThreshold;

if goalReached
    current = length(nodes);
    titleText = 'RRT Path Found';
else
    allNodes = vertcat(nodes.pos);
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

% --- Collision functions ---
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
