# Mobile Manipulation Simulation

A comprehensive MATLAB simulation for mobile manipulation robotics with interactive GUI, parameter tuning, and enhanced visualization capabilities.

## Features

### 🎮 Interactive GUI
- **Parameter Control**: Real-time adjustment of Kp and Ki gains using sliders
- **Simulation Control**: Run simulations and export results with one click
- **Enhanced Plots**: 6-panel visualization showing error components, trajectories, and performance metrics
- **Results Display**: Real-time performance metrics and simulation status

### 🔧 Parameter Tuning
- **Automated Optimization**: Tests 48 parameter combinations (8 Kp × 6 Ki values)
- **Performance Analysis**: Evaluates final error, max error, mean error, and RMS error
- **Visualization**: Surface plots and trend analysis for parameter optimization
- **Recommendations**: Automatic generation of optimal parameter suggestions

### 📊 Enhanced Output
- **Required Formats**: Generates Animation.csv, Traj.csv, and Xerr.mat files
- **Performance Reports**: Comprehensive simulation summaries with metrics
- **High-Resolution Plots**: PNG and PDF exports of all visualizations
- **Timestamped Files**: Unique file naming for result tracking

## Quick Start

### Launch GUI
```matlab
run_simulation()           % Launch interactive GUI
run_simulation('gui')      % Alternative GUI launch
```

### Command Line Options
```matlab
% Quick simulation with specific parameters
run_simulation('quick', 1.5, 0.0)    % Kp=1.5, Ki=0.0

% Run parameter tuning
run_simulation('tune')               % Test 48 parameter combinations

% Demonstration with optimal parameters
run_simulation('demo')               % Run with recommended settings

% Batch simulation with custom parameters
params = struct();
params.kp = 1.0;
params.ki = 0.4;
params.dt = 0.01;
params.T_total = 13;
params.maxspeed = 12.3;
run_simulation('batch', params)
```

## Optimal Parameters

Based on comprehensive parameter tuning, the optimal control parameters are:

| Metric | Kp | Ki | Final Error |
|--------|----|----|-------------|
| **Best Final Error** | 1.0 | 0.4 | 0.0006 |
| **Best Max Error** | 0.5 | 0.0 | 0.4056 |
| **Best Mean Error** | 4.0 | 0.0 | 0.0081 |
| **Best RMS Error** | 4.0 | 0.2 | 0.0402 |

### Top 5 Configurations
1. **Kp=1.0, Ki=0.4**: Final Error=0.0006 ⭐ **RECOMMENDED**
2. **Kp=4.0, Ki=0.0**: Final Error=0.0006
3. **Kp=3.5, Ki=0.0**: Final Error=0.0007
4. **Kp=3.0, Ki=0.0**: Final Error=0.0009
5. **Kp=1.0, Ki=0.5**: Final Error=0.0010

## File Structure

```
MIMIC_Capstone/
├── robotics_gui.m              # Main GUI application
├── run_simulation.m            # Launcher with multiple modes
├── parameter_tuning.m          # Automated parameter optimization
├── run_robotics_simulation.m   # Core simulation function
├── main_simulation.m           # Original simulation script
├── mobile_manipulator.m        # Mobile manipulator class
├── robot_base.m               # Base robot class
├── calculateTf.m              # Time allocation function
├── traj2mat.m                 # Trajectory conversion
├── rotx.m, roty.m, rotz.m     # Rotation functions
├── mr/                        # Modern Robotics library
└── README.md                  # This file
```

## Output Files

### Required Format Files
- `Animation.csv`: Robot configuration over time (13 columns)
- `Traj.csv`: Reference trajectory data (13 columns)
- `Xerr.mat`: Error twist data (6×N matrix)

### Enhanced Output Files
- `simulation_summary_YYYYMMDD_HHMMSS.txt`: Comprehensive performance report
- `parameters_YYYYMMDD_HHMMSS.txt`: Parameter configuration
- `simulation_plots_YYYYMMDD_HHMMSS.png/pdf`: High-resolution visualizations
- `parameter_tuning_results.mat`: Tuning analysis data
- `tuning_recommendations.mat`: Optimal parameter suggestions

## Performance Metrics

The simulation tracks multiple performance indicators:

- **Final Error**: Error magnitude at simulation end
- **Max Error**: Maximum error during simulation
- **Mean Error**: Average error over time
- **RMS Error**: Root mean square error
- **Simulation Time**: Computational performance

## GUI Features

### Parameter Panel
- **Kp Slider**: 0.1 to 5.0 (Proportional gain)
- **Ki Slider**: 0.0 to 2.0 (Integral gain)
- **Time Step**: Customizable simulation resolution
- **Total Time**: Simulation duration

### Control Panel
- **Run Simulation**: Execute with current parameters
- **Export Results**: Save all outputs with timestamps

### Results Panel
- **Real-time Status**: Simulation progress and completion
- **Performance Metrics**: Final, max, and mean errors
- **File Generation**: List of created output files

### Plot Panel
1. **Error Twist Components**: Wx, Wy, Wz, Vx, Vy, Vz
2. **End-Effector Position**: X, Y, Z coordinates over time
3. **Joint Angles**: 5-DOF manipulator joint states
4. **Total Error Magnitude**: Combined error over time
5. **Mobile Base Trajectory**: 2D position plot
6. **Grasp State**: Binary grasp indicator

## Technical Details

### Robot Configuration
- **Mobile Base**: 4-wheel mecanum drive
- **Manipulator**: 5-DOF articulated arm
- **End-Effector**: Gripper with grasp control
- **Control**: PI feedback control in task space

### Simulation Parameters
- **Time Step**: 0.01 seconds (default)
- **Total Time**: 13 seconds (default)
- **Max Speed**: 12.3 rad/s
- **Joint Limits**: ±π radians

### Control Law
```
V = Adjoint(T_inv) * Xd + Kp * xerr + Ki * ∫xerr dt
```

Where:
- `V`: Velocity command
- `Xd`: Desired velocity
- `xerr`: Error twist
- `Kp`: Proportional gain matrix
- `Ki`: Integral gain matrix

## Usage Examples

### Basic Simulation
```matlab
% Run with default parameters
run_simulation('quick', 1.5, 0.0)
```

### Parameter Optimization
```matlab
% Find optimal parameters
run_simulation('tune')

% Use recommended parameters
run_simulation('quick', 1.0, 0.4)
```

### Custom Parameters
```matlab
% Define custom parameters
params = struct();
params.kp = 2.0;
params.ki = 0.1;
params.dt = 0.005;
params.T_total = 15;
params.maxspeed = 15.0;

% Run batch simulation
run_simulation('batch', params)
```

## Troubleshooting

### Common Issues
1. **Path Issues**: Ensure `mr/` library is in the same directory
2. **Parameter Ranges**: Kp > 0, Ki ≥ 0, dt > 0, T_total > 0
3. **Memory**: Large parameter sweeps may require significant memory

### Performance Tips
- Use `dt = 0.01` for good balance of accuracy and speed
- Higher Kp values generally improve tracking but may cause oscillations
- Ki > 0 can reduce steady-state error but may slow response

## Dependencies

- MATLAB R2019b or later
- Modern Robotics library (`mr/` folder)
- Statistics and Machine Learning Toolbox (for RMS calculation)

## License

This project is for educational purposes. Please refer to the original Modern Robotics course materials for licensing information.

## Acknowledgments

Based on the Modern Robotics course materials from Northwestern University's Mechatronics Lab.
