function parameter_tuning()
    % Parameter Tuning Script for Mobile Manipulation Simulation
    % Automatically tests different Kp and Ki values to find optimal performance
    
    fprintf('Starting parameter tuning process...\n');
    
    % Define parameter ranges to test
    kp_range = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0];
    ki_range = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5];
    
    % Initialize results storage
    results = struct();
    results.kp_values = [];
    results.ki_values = [];
    results.final_errors = [];
    results.max_errors = [];
    results.mean_errors = [];
    results.rms_errors = [];
    results.simulation_times = [];
    results.success_flags = [];
    
    total_tests = length(kp_range) * length(ki_range);
    test_count = 0;
    
    fprintf('Testing %d parameter combinations...\n', total_tests);
    
    % Test all parameter combinations
    for i = 1:length(kp_range)
        for j = 1:length(ki_range)
            test_count = test_count + 1;
            kp_val = kp_range(i);
            ki_val = ki_range(j);
            
            fprintf('Test %d/%d: Kp=%.1f, Ki=%.1f\n', test_count, total_tests, kp_val, ki_val);
            
            try
                % Set parameters
                params = struct();
                params.kp = kp_val;
                params.ki = ki_val;
                params.dt = 0.01;
                params.T_total = 13;
                params.maxspeed = 12.3;
                
                % Run simulation
                tic;
                [Animation, Xerr, Traj, Td, grasp] = run_robotics_simulation(params);
                sim_time = toc;
                
                % Calculate performance metrics
                final_error = sqrt(sum(Xerr(:, end).^2));
                max_error = max(sqrt(sum(Xerr.^2, 1)));
                mean_error = mean(sqrt(sum(Xerr.^2, 1)));
                rms_error = rms(sqrt(sum(Xerr.^2, 1)));
                
                % Store results
                results.kp_values(end+1) = kp_val;
                results.ki_values(end+1) = ki_val;
                results.final_errors(end+1) = final_error;
                results.max_errors(end+1) = max_error;
                results.mean_errors(end+1) = mean_error;
                results.rms_errors(end+1) = rms_error;
                results.simulation_times(end+1) = sim_time;
                results.success_flags(end+1) = 1;
                
                fprintf('  Success: Final Error=%.4f, Max Error=%.4f, Time=%.2fs\n', ...
                        final_error, max_error, sim_time);
                
            catch ME
                fprintf('  Failed: %s\n', ME.message);
                
                % Store failed results
                results.kp_values(end+1) = kp_val;
                results.ki_values(end+1) = ki_val;
                results.final_errors(end+1) = inf;
                results.max_errors(end+1) = inf;
                results.mean_errors(end+1) = inf;
                results.rms_errors(end+1) = inf;
                results.simulation_times(end+1) = 0;
                results.success_flags(end+1) = 0;
            end
        end
    end
    
    % Analyze results
    analyze_tuning_results(results);
    
    % Save results
    save('parameter_tuning_results.mat', 'results');
    fprintf('Parameter tuning complete. Results saved to parameter_tuning_results.mat\n');
end

function analyze_tuning_results(results)
    % Analyze and display tuning results
    
    fprintf('\n=== PARAMETER TUNING RESULTS ===\n');
    
    % Find best parameters based on different criteria
    [~, best_final_idx] = min(results.final_errors);
    [~, best_max_idx] = min(results.max_errors);
    [~, best_mean_idx] = min(results.mean_errors);
    [~, best_rms_idx] = min(results.rms_errors);
    
    fprintf('\nBest Parameters by Final Error:\n');
    fprintf('  Kp=%.1f, Ki=%.1f: Final Error=%.4f\n', ...
            results.kp_values(best_final_idx), results.ki_values(best_final_idx), ...
            results.final_errors(best_final_idx));
    
    fprintf('\nBest Parameters by Max Error:\n');
    fprintf('  Kp=%.1f, Ki=%.1f: Max Error=%.4f\n', ...
            results.kp_values(best_max_idx), results.ki_values(best_max_idx), ...
            results.max_errors(best_max_idx));
    
    fprintf('\nBest Parameters by Mean Error:\n');
    fprintf('  Kp=%.1f, Ki=%.1f: Mean Error=%.4f\n', ...
            results.kp_values(best_mean_idx), results.ki_values(best_mean_idx), ...
            results.mean_errors(best_mean_idx));
    
    fprintf('\nBest Parameters by RMS Error:\n');
    fprintf('  Kp=%.1f, Ki=%.1f: RMS Error=%.4f\n', ...
            results.kp_values(best_rms_idx), results.ki_values(best_rms_idx), ...
            results.rms_errors(best_rms_idx));
    
    % Create performance visualization
    create_tuning_plots(results);
    
    % Generate recommendations
    generate_tuning_recommendations(results);
end

function create_tuning_plots(results)
    % Create visualization of tuning results
    
    figure('Name', 'Parameter Tuning Results', 'Position', [200, 200, 1200, 800]);
    
    % Create meshgrid for surface plots
    kp_unique = unique(results.kp_values);
    ki_unique = unique(results.ki_values);
    [KP, KI] = meshgrid(kp_unique, ki_unique);
    
    % Reshape results for surface plotting
    final_errors_matrix = reshape(results.final_errors, length(ki_unique), length(kp_unique));
    max_errors_matrix = reshape(results.max_errors, length(ki_unique), length(kp_unique));
    mean_errors_matrix = reshape(results.mean_errors, length(ki_unique), length(kp_unique));
    
    % Plot 1: Final Error Surface
    subplot(2, 3, 1);
    surf(KP, KI, final_errors_matrix);
    title('Final Error vs Parameters');
    xlabel('Kp'); ylabel('Ki'); zlabel('Final Error');
    colorbar;
    
    % Plot 2: Max Error Surface
    subplot(2, 3, 2);
    surf(KP, KI, max_errors_matrix);
    title('Max Error vs Parameters');
    xlabel('Kp'); ylabel('Ki'); zlabel('Max Error');
    colorbar;
    
    % Plot 3: Mean Error Surface
    subplot(2, 3, 3);
    surf(KP, KI, mean_errors_matrix);
    title('Mean Error vs Parameters');
    xlabel('Kp'); ylabel('Ki'); zlabel('Mean Error');
    colorbar;
    
    % Plot 4: Kp vs Final Error
    subplot(2, 3, 4);
    kp_final_errors = zeros(size(kp_unique));
    for i = 1:length(kp_unique)
        idx = find(results.kp_values == kp_unique(i) & results.ki_values == 0);
        if ~isempty(idx)
            kp_final_errors(i) = results.final_errors(idx);
        end
    end
    plot(kp_unique, kp_final_errors, 'b-o', 'LineWidth', 2);
    title('Final Error vs Kp (Ki=0)');
    xlabel('Kp'); ylabel('Final Error');
    grid on;
    
    % Plot 5: Ki vs Final Error (for best Kp)
    subplot(2, 3, 5);
    [~, best_kp_idx] = min(kp_final_errors);
    best_kp = kp_unique(best_kp_idx);
    ki_final_errors = zeros(size(ki_unique));
    for i = 1:length(ki_unique)
        idx = find(results.kp_values == best_kp & results.ki_values == ki_unique(i));
        if ~isempty(idx)
            ki_final_errors(i) = results.final_errors(idx);
        end
    end
    plot(ki_unique, ki_final_errors, 'r-o', 'LineWidth', 2);
    title(sprintf('Final Error vs Ki (Kp=%.1f)', best_kp));
    xlabel('Ki'); ylabel('Final Error');
    grid on;
    
    % Plot 6: Success rate
    subplot(2, 3, 6);
    success_rate = sum(results.success_flags) / length(results.success_flags) * 100;
    bar(1, success_rate, 'g');
    title('Simulation Success Rate');
    ylabel('Success Rate (%)');
    ylim([0, 100]);
    text(1, success_rate + 2, sprintf('%.1f%%', success_rate), ...
         'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    
    sgtitle('Parameter Tuning Analysis', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Save the plot
    saveas(gcf, 'parameter_tuning_analysis.png');
    saveas(gcf, 'parameter_tuning_analysis.pdf');
end

function generate_tuning_recommendations(results)
    % Generate tuning recommendations
    
    fprintf('\n=== TUNING RECOMMENDATIONS ===\n');
    
    % Find successful simulations
    success_idx = find(results.success_flags == 1);
    
    if isempty(success_idx)
        fprintf('No successful simulations found. Check parameter ranges.\n');
        return;
    end
    
    % Analyze successful results
    successful_final_errors = results.final_errors(success_idx);
    successful_kp = results.kp_values(success_idx);
    successful_ki = results.ki_values(success_idx);
    
    % Find top 5 best configurations
    [sorted_errors, sort_idx] = sort(successful_final_errors);
    top_5_idx = success_idx(sort_idx(1:min(5, length(sort_idx))));
    
    fprintf('\nTop 5 Parameter Configurations:\n');
    for i = 1:length(top_5_idx)
        idx = top_5_idx(i);
        fprintf('  %d. Kp=%.1f, Ki=%.1f: Final Error=%.4f\n', ...
                i, results.kp_values(idx), results.ki_values(idx), ...
                results.final_errors(idx));
    end
    
    % Analyze trends
    fprintf('\nParameter Trends:\n');
    
    % Kp trend
    kp_0_idx = find(successful_ki == 0);
    if ~isempty(kp_0_idx)
        [kp_errors, kp_sort] = sort(successful_final_errors(kp_0_idx));
        best_kp = successful_kp(kp_0_idx(kp_sort(1)));
        fprintf('  Best Kp (Ki=0): %.1f\n', best_kp);
    end
    
    % Ki trend
    if any(successful_ki > 0) && any(successful_ki == 0)
        ki_0_errors = successful_final_errors(successful_ki == 0);
        ki_nonzero_errors = successful_final_errors(successful_ki > 0);
        if ~isempty(ki_0_errors) && ~isempty(ki_nonzero_errors)
            ki_effect = mean(ki_nonzero_errors) - mean(ki_0_errors);
            if ki_effect < 0
                fprintf('  Ki generally improves performance\n');
            else
                fprintf('  Ki generally degrades performance\n');
            end
        end
    end
    
    % Generate final recommendation
    best_idx = top_5_idx(1);
    fprintf('\nRECOMMENDED PARAMETERS:\n');
    fprintf('  Kp = %.1f\n', results.kp_values(best_idx));
    fprintf('  Ki = %.1f\n', results.ki_values(best_idx));
    fprintf('  Expected Final Error: %.4f\n', results.final_errors(best_idx));
    
    % Save recommendations
    recommendations = struct();
    recommendations.best_kp = results.kp_values(best_idx);
    recommendations.best_ki = results.ki_values(best_idx);
    recommendations.expected_final_error = results.final_errors(best_idx);
    recommendations.top_5_configurations = [results.kp_values(top_5_idx)', ...
                                           results.ki_values(top_5_idx)', ...
                                           results.final_errors(top_5_idx)'];
    
    save('tuning_recommendations.mat', 'recommendations');
    fprintf('\nRecommendations saved to tuning_recommendations.mat\n');
end
