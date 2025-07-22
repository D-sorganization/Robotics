import csv
import heapq
import math
from collections import defaultdict

def read_nodes(filename):
    nodes = {}
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split(',')
            if len(parts) < 3:
                continue
            node_id = int(parts[0])
            x = float(parts[1])
            y = float(parts[2])
            nodes[node_id] = (x, y)
    return nodes

def read_edges(filename):
    graph = defaultdict(list)
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split(',')
            if len(parts) < 3:
                continue
            u = int(parts[0])
            v = int(parts[1])
            cost = float(parts[2])
            graph[u].append((v, cost))
            graph[v].append((u, cost))  # undirected graph
    return graph

def heuristic(a, b):
    # Euclidean distance
    return math.hypot(a[0] - b[0], a[1] - b[1])

def astar(start, goal, nodes, graph):
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {node: float('inf') for node in nodes}
    g_score[start] = 0

    f_score = {node: float('inf') for node in nodes}
    f_score[start] = heuristic(nodes[start], nodes[goal])

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor, cost in graph[current]:
            tentative_g = g_score[current] + cost
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(nodes[neighbor], nodes[goal])
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return [start]  # No path found

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]

def write_path(filename, path):
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(path)

def main():
    nodes_file = 'nodes.csv'
    edges_file = 'edges.csv'
    path_file = 'path.csv'

    nodes = read_nodes(nodes_file)
    graph = read_edges(edges_file)

    start_node = 1
    goal_node = max(nodes.keys())  # Assumes goal is highest node ID

    path = astar(start_node, goal_node, nodes, graph)
    write_path(path_file, path)

    print("Path found:" if len(path) > 1 else "No path found.")
    print(path)

if __name__ == '__main__':
    main()
