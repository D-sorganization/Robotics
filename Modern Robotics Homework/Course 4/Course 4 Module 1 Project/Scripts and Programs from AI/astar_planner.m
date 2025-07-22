function astar_planner()
    % File names
    nodes_file = 'nodes.csv';
    edges_file = 'edges.csv';
    path_file = 'path.csv';

    % Read input
    nodes = read_nodes(nodes_file);
    graph = read_edges(edges_file);

    start_node = 1;
    goal_node = size(nodes, 1);

    path = astar(start_node, goal_node, nodes, graph);
    
    % Write output
    write_path(path_file, path);

    if numel(path) > 1
        fprintf('Path found:\n');
    else
        fprintf('No path found.\n');
    end
    disp(path);
end

function nodes = read_nodes(filename)
    data = readmatrix(filename, 'CommentStyle', '#');
    nodes = data(:, 2:3);  % Ignore the node ID, assume it's ordered 1...N
end

function graph = read_edges(filename)
    data = readmatrix(filename, 'CommentStyle', '#');
    N = max(max(data(:,1:2)));  % Number of nodes
