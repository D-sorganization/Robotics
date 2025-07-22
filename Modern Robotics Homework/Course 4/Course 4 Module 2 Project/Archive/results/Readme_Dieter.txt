RRT Assignment using MATLAB
===========================

Description:
------------
This project implements the Rapidly-exploring Random Tree (RRT) algorithm for 2D path planning.
The algorithm begins at a predefined start position and expands a tree toward a goal position by
randomly sampling the workspace. When a node is generated that is within a specified threshold
of the goal, the algorithm backtracks to extract and record the final path.

Outputs:
---------
When executed, the script creates a folder named "results" in the same directory as the script.
Inside the "results" folder, the following CSV files are generated:

1. nodes.csv:
   Contains all nodes that were added to the RRT. Each row is formatted as:
   [NodeID, x-coordinate, y-coordinate, ParentNodeID].

2. edges.csv:
   Lists the parent-child associations for every node (excluding the root). Each row is [ChildNodeID, ParentNodeID].

3. path.csv:
   If the algorithm reaches the goal, this file contains the sequence of coordinates from the start to the goal.

Instructions:
-------------
1. Ensure you have MATLAB installed.
2. Place the files (rrt_assignment.m and README.txt) in your MATLAB working directory.
3. Run the script by typing "rrt_assignment" in the MATLAB command window or by opening the
   file in the MATLAB Editor and clicking the Run button.
4. Upon completion, locate the generated "results" folder. Open the CSV files with MATLAB or any 
   spreadsheet application (e.g., Excel) to review the node data, edge connections, and final path.

Notes:
------
- The current implementation uses a basic RRT approach without obstacles. If you require obstacle 
  avoidance, additional collision checking functions will need to be incorporated.
- You may modify parameters such as workspace boundaries, start/goal positions, step size, and the
  maximum number of iterations directly within the script if needed.

Enjoy exploring the RRT algorithm and feel free to customize the code to fit more advanced scenarios!