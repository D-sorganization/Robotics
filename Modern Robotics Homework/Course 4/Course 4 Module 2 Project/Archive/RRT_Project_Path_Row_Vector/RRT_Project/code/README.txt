# RRT Sampling-Based Planning Project

## Overview

This MATLAB implementation demonstrates the Rapidly-Exploring Random Tree (RRT) algorithm for motion planning in a 2D environment with circular obstacles. The robot is a point robot, and the planner finds a collision-free path from a start location to a goal location.

## Contents

- `main.m`: Entry point for running the RRT planner.
- `RRT.m`: Core RRT algorithm logic.
- `collisionCheck.m`: Collision checking between line segments and circular obstacles.
- `plotRRT.m`: Visualization of the RRT, obstacles, and path.
- `obstacles.csv`: Input file with circular obstacle information (center x, y, and **diameter**).
- `nodes.csv`: Output file containing all nodes in the tree.
- `edges.csv`: Output file of all edges in the tree.
- `path.csv`: Output file of the path from start to goal, if found.
- `obstacles_out.csv`: Output file with the same obstacle data used in the planner.
- `README.txt`: This file.

## How to Run

1. Open MATLAB and navigate to the directory.
2. Run the `main.m` script:
   ```matlab
   main
   ```
3. The script will:
   - Run the RRT planner.
   - Visualize the result.
   - Save outputs to CSV files.

## Sampling

Samples are drawn uniformly from the square [-0.5, 0.5] x [-0.5, 0.5] with 20% goal bias.

## Obstacles

Each row in `obstacles.csv` contains:
```
x_center, y_center, diameter
```
## Outputs

The following files are created after running the planner:
- `nodes.csv`: All sampled nodes with their parent index.
- `edges.csv`: Each edge from a node to its parent.
- `path.csv`: Final collision-free path from start to goal.
- `obstacles_out.csv`: Verified obstacle data used in computation.

## Visualization

The script `plotRRT.m` generates a plot showing:
- Black filled circles for obstacles.
- Blue lines for the RRT edges.
- A red line for the final path (if found).
- Green and red circles for the start and goal respectively.

## License

MIT License or educational use as permitted by Northwestern's Coursera course.
