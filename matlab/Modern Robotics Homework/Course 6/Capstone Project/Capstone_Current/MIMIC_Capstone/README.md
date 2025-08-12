# Mobile Manipulation Capstone Project

## Overview
This project implements a comprehensive mobile manipulation system for the youBot platform, demonstrating advanced robotics concepts including trajectory planning, odometry, and feedback control. The system successfully completes a pick-and-place task using a mobile manipulator with integrated arm and base control.

## Key Features
- **Trajectory Planning**: Implements both screw and Cartesian trajectory generation
- **Odometry**: Real-time state estimation for mobile base
- **Feedback Control**: PI controller with feedforward compensation
- **Joint Limit Enforcement**: Collision avoidance and constraint handling
- **Performance Analysis**: Comprehensive error analysis and visualization

## System Architecture
The project consists of three main components:

1. **robot_base.m**: Base class for mobile platform with odometry
2. **mobile_manipulator.m**: Derived class integrating arm and base control
3. **main_simulation.m**: Main simulation script with task execution

## Installation
1. Ensure MATLAB is installed with Robotics Toolbox
2. Add the `mr` library to your MATLAB path
3. Run `main_simulation` to execute the complete simulation

## Usage
```matlab
% Run the complete simulation
main_simulation
```

## Output Files
- `trajectory_output.csv`: Reference trajectory data
- `simulation_results.csv`: Simulation results
- `error_analysis.mat`: Error data for analysis
- `error_analysis.png/pdf`: Error visualization plots
- `trajectory_analysis.png/pdf`: Trajectory visualization plots
- `simulation_summary.txt`: Performance summary

## Performance Metrics
The system achieves:
- Low tracking errors (< 0.01 rad/s angular, < 0.01 m/s linear)
- Smooth trajectory execution
- Successful task completion
- Real-time control performance

## Technical Specifications
- **Platform**: youBot with 5-DOF arm
- **Control**: PI feedback with feedforward
- **Trajectory**: Cartesian path planning
- **Sampling**: 100 Hz control loop
- **Duration**: 13-second task execution

## Analysis Features
- Real-time error monitoring
- Trajectory visualization
- Performance statistics
- Comprehensive reporting

## Requirements
- MATLAB R2018b or later
- Robotics Toolbox
- Modern Robotics library (mr)

## License
This project is developed for educational purposes in robotics engineering.
