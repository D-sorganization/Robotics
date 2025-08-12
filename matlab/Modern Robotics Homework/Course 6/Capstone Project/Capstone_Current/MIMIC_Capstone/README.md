Mobile Manipulation Capstone Project
====================================

This project implements a mobile manipulation simulation for the youBot platform.
The robot performs a pick-and-place task using trajectory planning and feedback control.

Files:
- main_simulation.m: Main script to run the simulation
- robotics_gui.m: GUI for parameter tuning and visualization
- mobile_manipulator.m: Robot class with arm and base control
- robot_base.m: Base class for mobile platform
- calculateTf.m: Time allocation function
- traj2mat.m: Trajectory conversion
- rotx.m, roty.m, rotz.m: Rotation functions
- mr/: Modern Robotics library

How to Run:
1. Open MATLAB
2. Navigate to this folder
3. Run: main_simulation
   OR
   Run: robotics_gui (for interactive GUI)

GUI Features:
- Adjust Kp and Ki values using text boxes
- Run simulation with current parameters
- View plots of error, trajectory, and joint angles
- Export results to files
- Save/load parameter configurations

Output Files:
- Animation.csv: Robot configuration over time
- Traj.csv: Reference trajectory data  
- Xerr.mat: Error twist data
- results/ folder: All output files organized

Best Parameters Found:
- Kp = 1.0
- Ki = 0.4
- These give the lowest final error

Overshoot Case:
When Kp is too high (>3.0), the robot overshoots the target and oscillates.
This causes high error values and unstable behavior.

To avoid overshoot:
- Keep Kp between 0.5 and 2.0
- Use small Ki values (0.0 to 0.5)
- Test parameters using the GUI before running full simulation

Requirements:
- MATLAB R2019b or later
- Modern Robotics library (included in mr/ folder)

This project was completed for the Modern Robotics course.
