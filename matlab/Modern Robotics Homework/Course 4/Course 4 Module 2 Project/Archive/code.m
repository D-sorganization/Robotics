% run_rrt.m
% Main script for RRT motion planning in a 2D environment with circular obstacles

clear; clc;
cd(matlabdrive)
cd('_Coursera Homework/_Modern Robotics Course/Homework/Course 4/Course 4 Module 2 Project/')

% Load obstacle data
obstacles = readmatrix('results/obstacles.csv');

% Parameters
start = [-0.5, -0.5];
goal = [0.5, 0.5];
goalBias = 0.1;
stepSize = 0.05;
maxNodes = 1000;
goalThreshold = 0.05;

% Initialize tree with start node
tree.nodes = start;
tree.parents = 0;
foundPath = false;

for iter = 1:maxNodes
    q_rand = sampleFree(goal, goalBias);
    idx_near = nearestNode(tree.nodes, q_rand);
    q_near = tree.nodes(idx_near, :);
    q_new = steer(q_near, q_rand, stepSize);

    if collisionFree(q_near, q_new, obstacles)
        tree.nodes = [tree.nodes; q_new];
        tree.parents = [tree.parents; idx_near];

        if norm(q_new - goal) < goalThreshold
            tree.nodes = [tree.nodes; goal];
            tree.parents = [tree.parents; size(tree.nodes, 1) - 1];
            foundPath = true;
            break;
        end
    end
end

% Trace path from goal to start
path = [];
if foundPath
    idx = size(tree.nodes, 1);
    while idx ~= 0
        path = [tree.nodes(idx, :); path];
        idx = tree.parents(idx);
    end
end

% Write results
mkdir('results');
writeOutputFiles(tree.nodes, tree.parents, path);

%% ========== Functions ==========

function q_rand = sampleFree(goal, goalBias)
    if rand < goalBias
        q_rand = goal;
    else
        q_rand = -0.5 + rand(1, 2);
    end
end

function idx = nearestNode(nodes, q)
    dists = vecnorm(nodes - q, 2, 2);
    [~, idx] = min(dists);
end

function q_new = steer(q_near, q_rand, stepSize)
    dir = q_rand - q_near;
    dist = norm(dir);
    if dist <= stepSize
        q_new = q_rand;
    else
        q_new = q_near + stepSize * dir / dist;
    end
end

function free = collisionFree(q1, q2, obstacles)
    free = true;
    for i = 1:size(obstacles, 1)
        if intersectsCircle(q1, q2, obstacles(i, 1:2), obstacles(i, 3))
            free = false;
            return;
        end
    end
end

function hit = intersectsCircle(p1, p2, center, radius)
    d = p2 - p1;
    f = p1 - center;
    a = dot(d, d);
    b = 2 * dot(f, d);
    c = dot(f, f) - radius^2;
    discriminant = b^2 - 4*a*c;

    if discriminant < 0
        hit = false;
    else
        discriminant = sqrt(discriminant);
        t1 = (-b - discriminant) / (2*a);
        t2 = (-b + discriminant) / (2*a);
        hit = (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1);
    end
end

function writeOutputFiles(nodes, parents, path)
    writematrix(nodes, 'results/nodes.csv');

    edges = [];
    for i = 2:length(parents)
        edges = [edges; parents(i)-1, i-1]; % -1 for 0-based indexing in CoppeliaSim
    end
    writematrix(edges, 'results/edges.csv');

    if ~isempty(path)
        writematrix(path, 'results/path.csv');
    end
end
