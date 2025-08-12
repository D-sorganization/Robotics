function run_simulation(varargin)
    % Mobile Manipulation Simulation Launcher
    % Usage:
    %   run_simulation()                    - Launch GUI
    %   run_simulation('gui')               - Launch GUI
    %   run_simulation('tune')              - Run parameter tuning
    %   run_simulation('quick', kp, ki)     - Quick simulation with specified parameters
    %   run_simulation('batch', params)     - Batch simulation with parameter struct
    
    if nargin == 0
        % Default: launch GUI
        robotics_gui();
        return;
    end
    
    mode = varargin{1};
    
    switch lower(mode)
        case 'gui'
            robotics_gui();
            
        case 'tune'
            fprintf('Starting parameter tuning...\n');
            parameter_tuning();
            
        case 'quick'
            if nargin < 3
                error('Quick mode requires Kp and Ki values: run_simulation(''quick'', kp, ki)');
            end
            kp = varargin{2};
            ki = varargin{3};
            
            fprintf('Running quick simulation with Kp=%.2f, Ki=%.2f\n', kp, ki);
            
            params = struct();
            params.kp = kp;
            params.ki = ki;
            params.dt = 0.01;
            params.T_total = 13;
            params.maxspeed = 12.3;
            
            [Animation, Xerr, Traj, Td, grasp] = run_robotics_simulation(params);
            
            % Calculate performance metrics
            final_error = sqrt(sum(Xerr(:, end).^2));
            max_error = max(sqrt(sum(Xerr.^2, 1)));
            mean_error = mean(sqrt(sum(Xerr.^2, 1)));
            
            fprintf('Simulation complete!\n');
            fprintf('Final Error: %.4f\n', final_error);
            fprintf('Max Error: %.4f\n', max_error);
            fprintf('Mean Error: %.4f\n', mean_error);
            
            % Save results
            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
            writematrix(Animation, sprintf('Animation_quick_%s.csv', timestamp));
            writematrix(Traj, sprintf('Traj_quick_%s.csv', timestamp));
            save(sprintf('Xerr_quick_%s.mat', timestamp), 'Xerr');
            
            fprintf('Results saved with timestamp: %s\n', timestamp);
            
        case 'batch'
            if nargin < 2
                error('Batch mode requires parameter struct: run_simulation(''batch'', params)');
            end
            params = varargin{2};
            
            fprintf('Running batch simulation...\n');
            [Animation, Xerr, Traj, Td, grasp] = run_robotics_simulation(params);
            
            % Save results
            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
            writematrix(Animation, sprintf('Animation_batch_%s.csv', timestamp));
            writematrix(Traj, sprintf('Traj_batch_%s.csv', timestamp));
            save(sprintf('Xerr_batch_%s.mat', timestamp), 'Xerr');
            
            fprintf('Batch simulation complete. Results saved with timestamp: %s\n', timestamp);
            
        case 'demo'
            % Run a demonstration with optimal parameters
            fprintf('Running demonstration simulation...\n');
            
            % Use recommended parameters (can be updated based on tuning results)
            params = struct();
            params.kp = 1.5;  % Default optimal value
            params.ki = 0.0;  % Default optimal value
            params.dt = 0.01;
            params.T_total = 13;
            params.maxspeed = 12.3;
            
            [Animation, Xerr, Traj, Td, grasp] = run_robotics_simulation(params);
            
            % Create demonstration plots
            create_demo_plots(Animation, Xerr, Traj);
            
            % Save results
            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
            writematrix(Animation, sprintf('Animation_demo_%s.csv', timestamp));
            writematrix(Traj, sprintf('Traj_demo_%s.csv', timestamp));
            save(sprintf('Xerr_demo_%s.mat', timestamp), 'Xerr');
            
            fprintf('Demonstration complete. Results saved with timestamp: %s\n', timestamp);
            
        otherwise
            error('Unknown mode. Use: gui, tune, quick, batch, or demo');
    end
end

function create_demo_plots(Animation, Xerr, Traj)
    % Create demonstration plots
    
    figure('Name', 'Mobile Manipulation Demo', 'Position', [100, 100, 1400, 900]);
    
    % Plot 1: Error twist components
    subplot(3, 4, 1);
    plot(Xerr', 'LineWidth', 1.5);
    title('Error Twist Components');
    xlabel('Time Steps');
    ylabel('Error Magnitude');
    legend('Wx', 'Wy', 'Wz', 'Vx', 'Vy', 'Vz', 'Location', 'best');
    grid on;
    
    % Plot 2: End-effector trajectory
    subplot(3, 4, 2);
    time_steps = 1:size(Animation, 1);
    plot(time_steps, Animation(:, 4), 'b-', 'LineWidth', 2, 'DisplayName', 'X');
    hold on;
    plot(time_steps, Animation(:, 5), 'r-', 'LineWidth', 2, 'DisplayName', 'Y');
    plot(time_steps, Animation(:, 6), 'g-', 'LineWidth', 2, 'DisplayName', 'Z');
    title('End-Effector Position');
    xlabel('Time Steps');
    ylabel('Position (m)');
    legend('Location', 'best');
    grid on;
    
    % Plot 3: Joint angles
    subplot(3, 4, 3);
    plot(time_steps, Animation(:, 7), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Joint 1');
    hold on;
    plot(time_steps, Animation(:, 8), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Joint 2');
    plot(time_steps, Animation(:, 9), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Joint 3');
    plot(time_steps, Animation(:, 10), 'm-', 'LineWidth', 1.5, 'DisplayName', 'Joint 4');
    plot(time_steps, Animation(:, 11), 'c-', 'LineWidth', 1.5, 'DisplayName', 'Joint 5');
    title('Joint Angles');
    xlabel('Time Steps');
    ylabel('Angle (rad)');
    legend('Location', 'best');
    grid on;
    
    % Plot 4: Error magnitude over time
    subplot(3, 4, 4);
    error_magnitude = sqrt(sum(Xerr.^2, 1));
    plot(error_magnitude, 'LineWidth', 2, 'Color', 'red');
    title('Total Error Magnitude');
    xlabel('Time Steps');
    ylabel('Error Magnitude');
    grid on;
    
    % Plot 5: Mobile base trajectory
    subplot(3, 4, 5);
    plot(Animation(:, 2), Animation(:, 3), 'b-', 'LineWidth', 2);
    title('Mobile Base Trajectory');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    grid on;
    axis equal;
    
    % Plot 6: Grasp state
    subplot(3, 4, 6);
    plot(time_steps, Animation(:, 13), 'LineWidth', 2, 'Color', 'green');
    title('Grasp State');
    xlabel('Time Steps');
    ylabel('Grasp (0/1)');
    ylim([-0.1, 1.1]);
    grid on;
    
    % Plot 7: Mobile base orientation
    subplot(3, 4, 7);
    plot(time_steps, Animation(:, 1), 'LineWidth', 2, 'Color', 'blue');
    title('Mobile Base Orientation');
    xlabel('Time Steps');
    ylabel('Orientation (rad)');
    grid on;
    
    % Plot 8: Wheel angles
    subplot(3, 4, 8);
    plot(time_steps, Animation(:, 9), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Wheel 1');
    hold on;
    plot(time_steps, Animation(:, 10), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Wheel 2');
    plot(time_steps, Animation(:, 11), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Wheel 3');
    plot(time_steps, Animation(:, 12), 'm-', 'LineWidth', 1.5, 'DisplayName', 'Wheel 4');
    title('Wheel Angles');
    xlabel('Time Steps');
    ylabel('Angle (rad)');
    legend('Location', 'best');
    grid on;
    
    % Plot 9: 3D trajectory
    subplot(3, 4, [9, 10]);
    plot3(Animation(:, 4), Animation(:, 5), Animation(:, 6), 'b-', 'LineWidth', 2);
    title('3D End-Effector Trajectory');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on;
    axis equal;
    
    % Plot 10: Performance metrics
    subplot(3, 4, [11, 12]);
    final_error = sqrt(sum(Xerr(:, end).^2));
    max_error = max(sqrt(sum(Xerr.^2, 1)));
    mean_error = mean(sqrt(sum(Xerr.^2, 1)));
    
    metrics = [final_error, max_error, mean_error];
    metric_names = {'Final Error', 'Max Error', 'Mean Error'};
    bar(metrics, 'FaceColor', [0.3, 0.6, 0.9]);
    title('Performance Metrics');
    ylabel('Error Magnitude');
    set(gca, 'XTickLabel', metric_names);
    grid on;
    
    % Add text annotations
    for i = 1:length(metrics)
        text(i, metrics(i) + max(metrics)*0.05, sprintf('%.4f', metrics(i)), ...
             'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    
    sgtitle('Mobile Manipulation Simulation Demo', 'FontSize', 16, 'FontWeight', 'bold');
    
    % Save the demo plot
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    saveas(gcf, sprintf('demo_plots_%s.png', timestamp));
    saveas(gcf, sprintf('demo_plots_%s.pdf', timestamp));
end
