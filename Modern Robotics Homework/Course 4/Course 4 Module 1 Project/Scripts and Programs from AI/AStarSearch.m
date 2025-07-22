% AStarSearch.m
%
% This script implements the A* search algorithm to find a minimum-cost path 
% through an undirected weighted graph from a start node to a goal node.
%
% It expects the following CSV files in the working directory:
%   - nodes.csv: Contains node definitions; each non-comment line is:
%                id,x,y
%                Comments begin with '#' and are ignored.
%   - edges.csv: Contains edge definitions; each non-comment line is:
%                node1,node2,cost
%                (The graph is assumed to be undirected, so each edge is added bidirectionally.)
%
% By convention, node 1 is the start node and the node with the highest id is the goal.
% The computed solution path is written to 'path.csv' in the format:
%   1,node2,node3,...,N
%
% Clear the workspace.
clear; clc;

%% Input File Names
nodesFile = 'nodes.csv';
edgesFile = 'edges.csv';
outputFile = 'path.csv';

%% Read Nodes and Edges
nodes = readNodes(nodesFile);    % nodes is an N x 2 matrix; each row [x, y]
numNodes = size(nodes,1);
edgeData = readEdges(edgesFile);

% Build the graph as a cell array where graph{i} is an array of rows [neighbor, cost]
graph = cell(numNodes, 1);
for i = 1:size(edgeData, 1)
    n1 = edgeData(i, 1);
    n2 = edgeData(i, 2);
    cost = edgeData(i, 3);
    % Since the graph is undirected, add the connection both ways.
    graph{n1} = [graph{n1}; n2, cost];
    graph{n2} = [graph{n2}; n1, cost];
end

%% Define Start and Goal Nodes
startNode = 1;
goalNode  = numNodes;  % Assumes the highest node id is the goal

%% Run A* Search
path = astarSearch(graph, nodes, startNode, goalNode);

%% Write the Output Path
writePath(path, outputFile);
fprintf('A* search complete. The solution path has been saved in %s\n', outputFile);

%% ------------------- Function Definitions -----------------------------

function nodes = readNodes(filename)
% readNodes reads nodes from a CSV file.
% Expects non-comment lines of the form: id,x,y
% Returns an N x 2 matrix where each row gives [x, y] for node i.
    
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file: %s', filename);
    end
    nodes = [];  % We will grow this array according to the node id.
    % Process the file line by line.
    while ~feof(fid)
        tline = strtrim(fgetl(fid));
        if isempty(tline) || tline(1) == '#'
            continue;
        end
        data = strsplit(tline, ',');
        if length(data) >= 3
            nodeID = str2double(data{1});
            x = str2double(data{2});
            y = str2double(data{3});
            % Make sure the nodes matrix is large enough.
            if size(nodes,1) < nodeID
                nodes(nodeID, :) = [x, y];  %#ok<AGROW>
            else
                nodes(nodeID, :) = [x, y];
            end
        end
    end
    fclose(fid);
end

function edges = readEdges(filename)
% readEdges reads edges from a CSV file.
% Expects non-comment lines of the form: node1,node2,cost
% Returns a matrix where each row is [node1, node2, cost].

    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file: %s', filename);
    end
    edges = [];
    while ~feof(fid)
        tline = strtrim(fgetl(fid));
        if isempty(tline) || tline(1) == '#'
            continue;
        end
        data = strsplit(tline, ',');
        if length(data) >= 3
            n1 = str2double(data{1});
            n2 = str2double(data{2});
            cost = str2double(data{3});
            edges = [edges; n1, n2, cost];  %#ok<AGROW>
        end
    end
    fclose(fid);
end

function h = heuristic(node, goal, nodes)
% heuristic computes the Euclidean distance between node and goal.
% nodes is an N x 2 matrix with the coordinates of each node.
    pos = nodes(node, :);
    goalPos = nodes(goal, :);
    h = norm(goalPos - pos);
end

function path = astarSearch(graph, nodes, start, goal)
% astarSearch implements the A* search algorithm.
% Inputs:
%   graph - cell array where graph{i} = [neighbor, cost] (neighbors of node i)
%   nodes - N x 2 matrix with node coordinates
%   start - starting node index
%   goal  - goal node index
% Output:
%   path  - vector of node indices from start to goal (or just [start] if no path is found)

    numNodes = size(nodes,1);
    
    % Initialize relative cost values.
    gScore = inf(numNodes, 1);    % Cost from start to node
    gScore(start) = 0;
    
    fScore = inf(numNodes, 1);    % Estimated total cost (g + h)
    fScore(start) = heuristic(start, goal, nodes);
    
    % cameFrom will store the predecessor for each node in the optimal path.
    cameFrom = zeros(numNodes, 1);
    
    % openSet contains the set of nodes to be evaluated.
    openSet = start;
    
    % closedSet is a boolean vector tracking nodes that have been expanded.
    closedSet = false(numNodes, 1);
    
    while ~isempty(openSet)
        % Select the node in openSet with the lowest fScore.
        [~, idx] = min(fScore(openSet));
        current = openSet(idx);
        
        % If the goal is reached, reconstruct the path.
        if current == goal
            path = reconstructPath(cameFrom, current);
            return;
        end
        
        % Remove current node from the open set and mark it closed.
        openSet(idx) = [];
        closedSet(current) = true;
        
        % Process each neighbor of the current node.
        neighbors = graph{current};  % Each row is [neighbor, cost]
        for i = 1:size(neighbors,1)
            neighbor = neighbors(i, 1);
            cost = neighbors(i, 2);
            
            if closedSet(neighbor)
                continue;
            end
            
            tentative_gScore = gScore(current) + cost;
            if tentative_gScore < gScore(neighbor)
                cameFrom(neighbor) = current;
                gScore(neighbor) = tentative_gScore;
                fScore(neighbor) = tentative_gScore + heuristic(neighbor, goal, nodes);
                % Add neighbor to openSet if not already included.
                if ~ismember(neighbor, openSet)
                    openSet(end+1) = neighbor;  %#ok<AGROW>
                end
            end
        end
    end
    
    % If no path is found, return the start node only.
    path = start;
end

function path = reconstructPath(cameFrom, current)
% reconstructPath rebuilds the path by backtracking through the cameFrom map.
    path = current;
    while cameFrom(current) ~= 0
        current = cameFrom(current);
        path = [current, path];  % Prepend the predecessor.
    end
end

function writePath(path, filename)
% writePath writes the found path to a CSV file as a single line.
% The path vector will be output in the format: 1, node2, node3, ..., N
    fid = fopen(filename, 'w');
    if fid == -1
        error('Cannot open file for writing: %s', filename);
    end
    % Create a comma-separated string from the path vector.
    pathStr = num2str(path(1));
    for i = 2:length(path)
        pathStr = [pathStr, ',', num2str(path(i))];  %#ok<AGROW>
    end
    fprintf(fid, '%s\n', pathStr);
    fclose(fid);
end