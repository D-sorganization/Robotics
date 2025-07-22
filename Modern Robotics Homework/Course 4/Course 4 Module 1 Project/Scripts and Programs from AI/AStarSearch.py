import csv
import math
import heapq

def read_nodes(filename):
    """
    Reads nodes.csv and returns a dictionary mapping node ID to (x, y) coordinates.
    Expects each non-commented line to have: id,x,y
    """
    nodes = {}
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            # Skip any comment lines (starting with "#")
            if row and row[0].startswith('#'):
                continue
            if len(row) < 3:
                continue
            node_id = int(row[0])
            x = float(row[1])
            y = float(row[2])
            nodes[node_id] = (x, y)
    return nodes

def read_edges(filename):
    """
    Reads edges.csv and constructs the graph as an adjacency dictionary.
    Expects each non-commented line: node1,node2,cost
    Since the graph is undirected, add edges both ways.
    """
    graph = {}
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if row and row[0].startswith('#'):
                continue
            if len(row) < 3:
                continue
            node1 = int(row[0])
            node2 = int(row[1])
            cost = float(row[2])
            if node1 not in graph:
                graph[node1] = []
            if node2 not in graph:
                graph[node2] = []
            graph[node1].append((node2, cost))
            graph[node2].append((node1, cost))
    return graph

def heuristic(node, goal, nodes):
    """
    Euclidean distance between the current node and the goal.
    """
    (x1, y1) = nodes[node]
    (x2, y2) = nodes[goal]
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def astar_search(nodes, graph, start, goal):
    """
    Implements the A* search algorithm.
    Returns a list of node IDs representing the solution path, or None if no path exists.
    """
    # Priority queue: each item is (f_score, node)
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal, nodes), start))
    
    came_from = {}  # Map each node to its predecessor in optimal path
    g_score = {node: float('inf') for node in nodes}
    g_score[start] = 0

    closed_set = set()

    while open_set:
        current_f, current = heapq.heappop(open_set)
        
        # If we reached the goal, reconstruct the path
        if current == goal:
            return reconstruct_path(came_from, current)

        closed_set.add(current)

        for neighbor, cost in graph.get(current, []):
            if neighbor in closed_set:
                continue
            tentative_g = g_score[current] + cost
            
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal, nodes)
                heapq.heappush(open_set, (f_score, neighbor))
    return None

def reconstruct_path(came_from, current):
    """
    Reconstructs the path from the start node to the current node.
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def write_path(path, filename):
    """
    Writes the solution path to a CSV file. If no path is found, writes just "1".
    """
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        if path is None:
            writer.writerow([1])
        else:
            writer.writerow(path)

if __name__ == "__main__":
    # Read the input files
    nodes = read_nodes('nodes.csv')
    graph = read_edges('edges.csv')

    # Assumption: Start is node 1, and goal is the node with the highest number.
    start = 1
    goal = max(nodes.keys())

    # Run the A* search
    path = astar_search(nodes, graph, start, goal)

    # Write the output path into path.csv
    write_path(path, 'path.csv')