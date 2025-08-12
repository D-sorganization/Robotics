function robotics_gui()
    % Robotics Simulation GUI
    % Provides interactive control over simulation parameters and visualization
    
    % Create main figure
    fig = figure('Name', 'Mobile Manipulation Simulation', ...
                 'Position', [100, 100, 1200, 800], ...
                 'MenuBar', 'none', ...
                 'ToolBar', 'none', ...
                 'Resize', 'on');
    
    % Initialize default parameters
    params = struct();
    params.kp = 1.0;  % Updated to optimal value
    params.ki = 0.4;  % Updated to optimal value
    params.dt = 0.01;
    params.T_total = 13;
    params.maxspeed = 12.3;
    
    % Create UI components
    create_ui_components(fig, params);
    
    % Store parameters in figure
    setappdata(fig, 'params', params);
    
    % Initialize results storage
    setappdata(fig, 'results', []);
    setappdata(fig, 'simulation_complete', false);
end

function create_ui_components(fig, params)
    % Create parameter panel
    param_panel = uipanel(fig, 'Title', 'Simulation Parameters', ...
                         'Position', [0.02, 0.7, 0.25, 0.28], ...
                         'FontSize', 12, 'FontWeight', 'bold');
    
    % Kp text box
    uicontrol(param_panel, 'Style', 'text', 'String', 'Proportional Gain (Kp):', ...
              'Position', [10, 180, 150, 20], 'HorizontalAlignment', 'left');
    kp_edit = uicontrol(param_panel, 'Style', 'edit', ...
                        'String', num2str(params.kp), ...
                        'Position', [10, 160, 80, 20], ...
                        'Callback', @(src,~) update_kp(src));
    
    % Ki text box
    uicontrol(param_panel, 'Style', 'text', 'String', 'Integral Gain (Ki):', ...
              'Position', [10, 130, 150, 20], 'HorizontalAlignment', 'left');
    ki_edit = uicontrol(param_panel, 'Style', 'edit', ...
                        'String', num2str(params.ki), ...
                        'Position', [10, 110, 80, 20], ...
                        'Callback', @(src,~) update_ki(src));
    
    % Time step
    uicontrol(param_panel, 'Style', 'text', 'String', 'Time Step (dt):', ...
              'Position', [10, 80, 150, 20], 'HorizontalAlignment', 'left');
    dt_edit = uicontrol(param_panel, 'Style', 'edit', ...
                        'String', num2str(params.dt), ...
                        'Position', [10, 60, 80, 20], ...
                        'Callback', @(src,~) update_dt(src));
    
    % Total time
    uicontrol(param_panel, 'Style', 'text', 'String', 'Total Time (s):', ...
              'Position', [10, 30, 150, 20], 'HorizontalAlignment', 'left');
    T_edit = uicontrol(param_panel, 'Style', 'edit', ...
                       'String', num2str(params.T_total), ...
                       'Position', [10, 10, 80, 20], ...
                       'Callback', @(src,~) update_T_total(src));
    
    % Control panel
    control_panel = uipanel(fig, 'Title', 'Simulation Control', ...
                           'Position', [0.02, 0.55, 0.25, 0.13], ...
                           'FontSize', 12, 'FontWeight', 'bold');
    
    % Run simulation button
    run_btn = uicontrol(control_panel, 'Style', 'pushbutton', ...
                        'String', 'Run Simulation', ...
                        'Position', [10, 40, 100, 30], ...
                        'Callback', @(src,~) run_simulation(fig, kp_edit, ki_edit, dt_edit, T_edit));
    
    % Export results button
    export_btn = uicontrol(control_panel, 'Style', 'pushbutton', ...
                           'String', 'Export Results', ...
                           'Position', [120, 40, 100, 30], ...
                           'Callback', @(src,~) export_results(fig), ...
                           'Enable', 'off');
    
    % Save/Load panel
    save_panel = uipanel(fig, 'Title', 'Save/Load Parameters', ...
                        'Position', [0.02, 0.45, 0.25, 0.08], ...
                        'FontSize', 12, 'FontWeight', 'bold');
    
    % Save parameters button
    save_btn = uicontrol(save_panel, 'Style', 'pushbutton', ...
                         'String', 'Save Best', ...
                         'Position', [10, 10, 80, 25], ...
                         'Callback', @(src,~) save_best_parameters(fig, kp_edit, ki_edit));
    
    % Save custom button
    save_custom_btn = uicontrol(save_panel, 'Style', 'pushbutton', ...
                                'String', 'Save Custom', ...
                                'Position', [100, 10, 80, 25], ...
                                'Callback', @(src,~) save_custom_parameters(fig, kp_edit, ki_edit, dt_edit, T_edit));
    
    % Load button
    load_btn = uicontrol(save_panel, 'Style', 'pushbutton', ...
                         'String', 'Load', ...
                         'Position', [190, 10, 60, 25], ...
                         'Callback', @(src,~) load_parameters(fig, kp_edit, ki_edit, dt_edit, T_edit));
    
    % Results panel
    results_panel = uipanel(fig, 'Title', 'Simulation Results', ...
                           'Position', [0.02, 0.02, 0.25, 0.41], ...
                           'FontSize', 12, 'FontWeight', 'bold');
    
    % Results text area
    results_text = uicontrol(results_panel, 'Style', 'text', ...
                             'String', 'No simulation run yet.', ...
                             'Position', [10, 10, 200, 200], ...
                             'HorizontalAlignment', 'left', ...
                             'FontSize', 10);
    
    % Plot panel
    plot_panel = uipanel(fig, 'Title', 'Simulation Plots', ...
                        'Position', [0.3, 0.02, 0.68, 0.96], ...
                        'FontSize', 12, 'FontWeight', 'bold');
    
    % Store UI handles
    handles = struct();
    handles.kp_edit = kp_edit;
    handles.ki_edit = ki_edit;
    handles.dt_edit = dt_edit;
    handles.T_edit = T_edit;
    handles.run_btn = run_btn;
    handles.export_btn = export_btn;
    handles.results_text = results_text;
    handles.plot_panel = plot_panel;
    
    setappdata(fig, 'handles', handles);
end

function update_kp(edit)
    % Validation will be done in run_simulation
end

function update_ki(edit)
    % Validation will be done in run_simulation
end

function update_dt(edit)
    % Validation will be done in run_simulation
end

function update_T_total(edit)
    % Validation will be done in run_simulation
end

function save_best_parameters(fig, kp_edit, ki_edit)
    % Save the optimal parameters to the main simulation script
    try
        % Read the main simulation file
        filename = 'main_simulation.m';
        fid = fopen(filename, 'r');
        if fid == -1
            errordlg('Could not open main_simulation.m', 'Save Error');
            return;
        end
        
        content = textscan(fid, '%s', 'Delimiter', '\n', 'Whitespace', '');
        fclose(fid);
        lines = content{1};
        
        % Get current values
        kp_val = str2double(get(kp_edit, 'String'));
        ki_val = str2double(get(ki_edit, 'String'));
        
        % Find and replace the Kp and Ki lines
        for i = 1:length(lines)
            if contains(lines{i}, 'Mybot.kp =') && contains(lines{i}, 'eye(6)')
                lines{i} = sprintf('Mybot.kp = %.1f * eye(6);  % Updated to optimal value', kp_val);
            elseif contains(lines{i}, 'Mybot.ki =') && contains(lines{i}, 'eye(6)')
                lines{i} = sprintf('Mybot.ki = %.1f * eye(6);  % Updated to optimal value', ki_val);
            end
        end
        
        % Write back to file
        fid = fopen(filename, 'w');
        for i = 1:length(lines)
            fprintf(fid, '%s\n', lines{i});
        end
        fclose(fid);
        
        msgbox(sprintf('Best parameters saved to main_simulation.m\nKp = %.1f, Ki = %.1f', kp_val, ki_val), 'Save Complete');
        
    catch ME
        errordlg(['Error saving parameters: ' ME.message], 'Save Error');
    end
end

function save_custom_parameters(fig, kp_edit, ki_edit, dt_edit, T_edit)
    % Save custom parameters to a file
    try
        % Get current values
        kp_val = str2double(get(kp_edit, 'String'));
        ki_val = str2double(get(ki_edit, 'String'));
        dt_val = str2double(get(dt_edit, 'String'));
        T_val = str2double(get(T_edit, 'String'));
        
        % Validate values
        if isnan(kp_val) || isnan(ki_val) || isnan(dt_val) || isnan(T_val)
            errordlg('Invalid parameter values', 'Save Error');
            return;
        end
        
        % Create parameter struct
        params = struct();
        params.kp = kp_val;
        params.ki = ki_val;
        params.dt = dt_val;
        params.T_total = T_val;
        params.maxspeed = 12.3;
        
        % Get filename from user
        [filename, pathname] = uiputfile('*.mat', 'Save Parameters As');
        if filename == 0
            return;
        end
        
        % Save parameters
        fullpath = fullfile(pathname, filename);
        save(fullpath, 'params');
        
        msgbox(sprintf('Parameters saved to %s', filename), 'Save Complete');
        
    catch ME
        errordlg(['Error saving parameters: ' ME.message], 'Save Error');
    end
end

function load_parameters(fig, kp_edit, ki_edit, dt_edit, T_edit)
    % Load parameters from a file
    try
        % Get filename from user
        [filename, pathname] = uigetfile('*.mat', 'Load Parameters');
        if filename == 0
            return;
        end
        
        % Load parameters
        fullpath = fullfile(pathname, filename);
        data = load(fullpath);
        
        if isfield(data, 'params')
            params = data.params;
            
            % Update GUI
            set(kp_edit, 'String', num2str(params.kp));
            set(ki_edit, 'String', num2str(params.ki));
            set(dt_edit, 'String', num2str(params.dt));
            set(T_edit, 'String', num2str(params.T_total));
            
            msgbox(sprintf('Parameters loaded from %s', filename), 'Load Complete');
        else
            errordlg('Invalid parameter file', 'Load Error');
        end
        
    catch ME
        errordlg(['Error loading parameters: ' ME.message], 'Load Error');
    end
end

function run_simulation(fig, kp_edit, ki_edit, dt_edit, T_edit)
    % Get current parameters
    kp_val = str2double(get(kp_edit, 'String'));
    ki_val = str2double(get(ki_edit, 'String'));
    dt_val = str2double(get(dt_edit, 'String'));
    T_val = str2double(get(T_edit, 'String'));
    
    % Validate parameters
    if isnan(dt_val) || dt_val <= 0
        errordlg('Invalid time step. Must be positive number.', 'Parameter Error');
        return;
    end
    if isnan(T_val) || T_val <= 0
        errordlg('Invalid total time. Must be positive number.', 'Parameter Error');
        return;
    end
    if isnan(kp_val) || kp_val <= 0
        errordlg('Invalid Kp. Must be positive number.', 'Parameter Error');
        return;
    end
    if isnan(ki_val) || ki_val < 0
        errordlg('Invalid Ki. Must be non-negative number.', 'Parameter Error');
        return;
    end
    
    % Update parameters
    params = getappdata(fig, 'params');
    params.kp = kp_val;
    params.ki = ki_val;
    params.dt = dt_val;
    params.T_total = T_val;
    setappdata(fig, 'params', params);
    
    % Get handles
    handles = getappdata(fig, 'handles');
    
    % Update status
    set(handles.results_text, 'String', 'Running simulation...');
    set(handles.run_btn, 'Enable', 'off');
    drawnow;
    
    try
        % Run the simulation
        [Animation, Xerr, Traj, Td, grasp] = run_robotics_simulation(params);
        
        % Store results
        results = struct();
        results.Animation = Animation;
        results.Xerr = Xerr;
        results.Traj = Traj;
        results.Td = Td;
        results.grasp = grasp;
        results.params = params;
        setappdata(fig, 'results', results);
        setappdata(fig, 'simulation_complete', true);
        
        % Update plots
        create_enhanced_plots(fig, results);
        
        % Update results display
        update_results_display(fig, results);
        
        % Enable export button
        set(handles.export_btn, 'Enable', 'on');
        
    catch ME
        errordlg(['Simulation failed: ' ME.message], 'Simulation Error');
    end
    
    % Re-enable run button
    set(handles.run_btn, 'Enable', 'on');
end

function [Animation, Xerr, Traj, Td, grasp] = run_robotics_simulation(params)
    % Add the Functions library to path
    addpath(genpath('Functions'));
    
    % mobile base parameters
    l = 0.47/2;
    w = 0.3/2;
    r = 0.0475;
    T_b0 = RpToTrans(eye(3),[0.1662,0,0.0026]');
    M_0e = RpToTrans(eye(3),[0.033,0,0.6546]');
    Blists = [[0,0,1,0,0.033,0]',[0,-1,0,-0.5076,0,0]',[0,-1,0,-0.3526,0,0]'...
        [0,-1,0,-0.2176,0,0]',[0,0,1,0,0,0]'];
    
    % cube configuration
    T_sc_initial = RpToTrans(eye(3),[1,0,0.025]');
    T_sc_goal = RpToTrans(rotz(-pi/2),[0,-1,0.025]');
    a = pi/5;
    T_ce_standoff = [[-sin(a),0,-cos(a),0]',[0,1,0,0]',[cos(a),0,-sin(a),0]',...
        [0,0,0.25,1]'];
    T_ce_grasp = [[-sin(a),0,-cos(a),0]',[0,1,0,0]',[cos(a),0,-sin(a),0]',...
        [0,0,0,1]'];
    
    % end-effector planned configuration(reference) 
    T_se_initial = [0,0,1,0;0,1,0,0;-1,0,0,0.5;0,0,0,1];
    T_standoff_initial = T_sc_initial * T_ce_standoff;
    T_grasp = T_sc_initial * T_ce_grasp;
    T_standoff_final = T_sc_goal * T_ce_standoff;
    T_release = T_sc_goal * T_ce_grasp;
    
    %Construct a cell array for the path
    T_configure = {T_se_initial,T_standoff_initial,T_grasp,T_grasp,...
        T_standoff_initial,T_standoff_final,T_release,T_release,...
        T_standoff_final};
    
    % Create the robot object Mybot
    Mybot = mobile_manipulator(l,w,r,T_b0,M_0e,Blists);
    
    % Generating reference trajectory
    dt = params.dt;
    T_total = params.T_total;
    Tf = calculateTf(T_total);
    Traj = [];
    grasp_state = 0;
    for i = 1:8
        if i == 3 
            grasp_state = 1;
        elseif i == 7
            grasp_state = 0;
        end
        
        Trajectory = Mybot.TrajectoryGenerator(T_configure{i},...
            T_configure{i+1},Tf(i),dt,grasp_state,'Cartesian',5);
        Traj = [Traj;Trajectory];
    end
    
    % Set controller parameters
    Mybot.q = [0,0,0];
    Mybot.theta = [0,0,0.2,-1.67,0]';
    Mybot.wheelAngle = [pi/2,pi/2,pi/2,pi/2];
    Mybot.kp = params.kp * eye(6);
    Mybot.ki = params.ki * eye(6);
    maxspeed = params.maxspeed * ones(1,9);
    jointLimits = [[pi,-pi]',[pi,-pi]',[pi,-pi]',[pi,-pi]',[pi,-pi]'];
    
    [Td,grasp] = traj2mat(Traj);
    
    % Apply feedback control
    [Animation,Xerr] = Mybot.FeedbackControl(dt,Td,maxspeed,grasp,jointLimits);
    
    % Clean up temporary variables (keep only output variables)
    clear l w r T_b0 M_0e Blists T_sc_initial T_sc_goal a T_ce_standoff T_ce_grasp;
    clear T_se_initial T_standoff_initial T_grasp T_standoff_final T_release;
    clear T_configure Mybot dt T_total Tf grasp_state Trajectory;
    clear maxspeed jointLimits;
end

function create_enhanced_plots(fig, results)
    handles = getappdata(fig, 'handles');
    plot_panel = handles.plot_panel;
    
    % Clear existing plots
    delete(get(plot_panel, 'Children'));
    
    % Create subplots
    subplot1 = subplot(2, 3, 1, 'Parent', plot_panel);
    subplot2 = subplot(2, 3, 2, 'Parent', plot_panel);
    subplot3 = subplot(2, 3, 3, 'Parent', plot_panel);
    subplot4 = subplot(2, 3, 4, 'Parent', plot_panel);
    subplot5 = subplot(2, 3, 5, 'Parent', plot_panel);
    subplot6 = subplot(2, 3, 6, 'Parent', plot_panel);
    
    % Plot 1: Error twist components
    plot(subplot1, results.Xerr', 'LineWidth', 1.5);
    title(subplot1, 'Error Twist Components', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel(subplot1, 'Time Steps', 'FontSize', 10);
    ylabel(subplot1, 'Error Magnitude', 'FontSize', 10);
    legend(subplot1, 'Wx', 'Wy', 'Wz', 'Vx', 'Vy', 'Vz', 'Location', 'best');
    grid(subplot1, 'on');
    
    % Plot 2: End-effector trajectory
    time_steps = 1:size(results.Animation, 1);
    plot(subplot2, time_steps, results.Animation(:, 4), 'b-', 'LineWidth', 2, 'DisplayName', 'X');
    hold(subplot2, 'on');
    plot(subplot2, time_steps, results.Animation(:, 5), 'r-', 'LineWidth', 2, 'DisplayName', 'Y');
    plot(subplot2, time_steps, results.Animation(:, 6), 'g-', 'LineWidth', 2, 'DisplayName', 'Z');
    title(subplot2, 'End-Effector Position', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel(subplot2, 'Time Steps', 'FontSize', 10);
    ylabel(subplot2, 'Position (m)', 'FontSize', 10);
    legend(subplot2, 'Location', 'best');
    grid(subplot2, 'on');
    
    % Plot 3: Joint angles
    plot(subplot3, time_steps, results.Animation(:, 7), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Joint 1');
    hold(subplot3, 'on');
    plot(subplot3, time_steps, results.Animation(:, 8), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Joint 2');
    plot(subplot3, time_steps, results.Animation(:, 9), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Joint 3');
    plot(subplot3, time_steps, results.Animation(:, 10), 'm-', 'LineWidth', 1.5, 'DisplayName', 'Joint 4');
    plot(subplot3, time_steps, results.Animation(:, 11), 'c-', 'LineWidth', 1.5, 'DisplayName', 'Joint 5');
    title(subplot3, 'Joint Angles', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel(subplot3, 'Time Steps', 'FontSize', 10);
    ylabel(subplot3, 'Angle (rad)', 'FontSize', 10);
    legend(subplot3, 'Location', 'best');
    grid(subplot3, 'on');
    
    % Plot 4: Error magnitude over time
    error_magnitude = sqrt(sum(results.Xerr.^2, 1));
    plot(subplot4, error_magnitude, 'LineWidth', 2, 'Color', 'red');
    title(subplot4, 'Total Error Magnitude', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel(subplot4, 'Time Steps', 'FontSize', 10);
    ylabel(subplot4, 'Error Magnitude', 'FontSize', 10);
    grid(subplot4, 'on');
    
    % Plot 5: Mobile base position
    plot(subplot5, results.Animation(:, 2), results.Animation(:, 3), 'b-', 'LineWidth', 2);
    title(subplot5, 'Mobile Base Trajectory', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel(subplot5, 'X Position (m)', 'FontSize', 10);
    ylabel(subplot5, 'Y Position (m)', 'FontSize', 10);
    grid(subplot5, 'on');
    axis(subplot5, 'equal');
    
    % Plot 6: Grasp state
    plot(subplot6, time_steps, results.Animation(:, 13), 'LineWidth', 2, 'Color', 'green');
    title(subplot6, 'Grasp State', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel(subplot6, 'Time Steps', 'FontSize', 10);
    ylabel(subplot6, 'Grasp (0/1)', 'FontSize', 10);
    ylim(subplot6, [-0.1, 1.1]);
    grid(subplot6, 'on');
    
    % Adjust layout
    sgtitle(plot_panel, 'Mobile Manipulation Simulation Results', 'FontSize', 14, 'FontWeight', 'bold');
end

function update_results_display(fig, results)
    handles = getappdata(fig, 'handles');
    
    % Calculate performance metrics
    final_error = sqrt(sum(results.Xerr(:, end).^2));
    max_error = max(sqrt(sum(results.Xerr.^2, 1)));
    mean_error = mean(sqrt(sum(results.Xerr.^2, 1)));
    
    % Create results summary
    summary = sprintf(['Simulation Complete!\n\n' ...
                      'Parameters:\n' ...
                      '  Kp: %.2f\n' ...
                      '  Ki: %.2f\n' ...
                      '  dt: %.3f\n' ...
                      '  Total Time: %.1f s\n\n' ...
                      'Performance:\n' ...
                      '  Final Error: %.4f\n' ...
                      '  Max Error: %.4f\n' ...
                      '  Mean Error: %.4f\n' ...
                      '  Simulation Steps: %d\n\n' ...
                      'Files Generated:\n' ...
                      '  - results/Animation.csv\n' ...
                      '  - results/Traj.csv\n' ...
                      '  - results/Xerr.mat\n' ...
                      '  - results/performance_summary.txt'], ...
                      results.params.kp, results.params.ki, ...
                      results.params.dt, results.params.T_total, ...
                      final_error, max_error, mean_error, ...
                      size(results.Animation, 1));
    
    set(handles.results_text, 'String', summary);
end

function export_results(fig)
    results = getappdata(fig, 'results');
    if isempty(results)
        errordlg('No results to export. Please run simulation first.', 'Export Error');
        return;
    end
    
    % Create results folder if it doesn't exist
    if ~exist('results', 'dir')
        mkdir('results');
    end
    
    % Create timestamp for unique filenames
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    
    % Export Animation.csv (required format)
    writematrix(results.Animation, sprintf('results/Animation_%s.csv', timestamp));
    
    % Export Traj.csv (required format)
    writematrix(results.Traj, sprintf('results/Traj_%s.csv', timestamp));
    
    % Export Xerr.mat (required format)
    Xerr = results.Xerr;
    save(sprintf('results/Xerr_%s.mat', timestamp), 'Xerr');
    
    % Export enhanced results summary
    create_results_summary(results, timestamp);
    
    % Export parameter configuration
    create_parameter_report(results.params, timestamp);
    
    % Export plots as images
    save_plots_as_images(fig, timestamp);
    
    msgbox(sprintf('Results exported successfully!\nFiles saved to results/ folder with timestamp: %s', timestamp), 'Export Complete');
end

function create_results_summary(results, timestamp)
    % Calculate comprehensive performance metrics
    final_error = sqrt(sum(results.Xerr(:, end).^2));
    max_error = max(sqrt(sum(results.Xerr.^2, 1)));
    mean_error = mean(sqrt(sum(results.Xerr.^2, 1)));
    rms_error = rms(sqrt(sum(results.Xerr.^2, 1)));
    
    % Create summary file
    filename = sprintf('results/simulation_summary_%s.txt', timestamp);
    fid = fopen(filename, 'w');
    
    fprintf(fid, 'MOBILE MANIPULATION SIMULATION RESULTS\n');
    fprintf(fid, '=====================================\n\n');
    fprintf(fid, 'Simulation Date: %s\n', datestr(now));
    fprintf(fid, 'Timestamp: %s\n\n', timestamp);
    
    fprintf(fid, 'CONTROL PARAMETERS:\n');
    fprintf(fid, '-------------------\n');
    fprintf(fid, 'Proportional Gain (Kp): %.3f\n', results.params.kp);
    fprintf(fid, 'Integral Gain (Ki): %.3f\n', results.params.ki);
    fprintf(fid, 'Time Step (dt): %.3f s\n', results.params.dt);
    fprintf(fid, 'Total Simulation Time: %.1f s\n', results.params.T_total);
    fprintf(fid, 'Maximum Speed: %.1f rad/s\n\n', results.params.maxspeed);
    
    fprintf(fid, 'PERFORMANCE METRICS:\n');
    fprintf(fid, '-------------------\n');
    fprintf(fid, 'Final Error Magnitude: %.6f\n', final_error);
    fprintf(fid, 'Maximum Error Magnitude: %.6f\n', max_error);
    fprintf(fid, 'Mean Error Magnitude: %.6f\n', mean_error);
    fprintf(fid, 'RMS Error Magnitude: %.6f\n', rms_error);
    fprintf(fid, 'Total Simulation Steps: %d\n\n', size(results.Animation, 1));
    
    fprintf(fid, 'TRAJECTORY ANALYSIS:\n');
    fprintf(fid, '-------------------\n');
    fprintf(fid, 'End-Effector Final Position: [%.3f, %.3f, %.3f] m\n', ...
            results.Animation(end, 4), results.Animation(end, 5), results.Animation(end, 6));
    fprintf(fid, 'Mobile Base Final Position: [%.3f, %.3f] m\n', ...
            results.Animation(end, 2), results.Animation(end, 3));
    fprintf(fid, 'Mobile Base Final Orientation: %.3f rad\n', results.Animation(end, 1));
    
    fclose(fid);
end

function create_parameter_report(params, timestamp)
    filename = sprintf('results/parameters_%s.txt', timestamp);
    fid = fopen(filename, 'w');
    
    fprintf(fid, 'SIMULATION PARAMETERS\n');
    fprintf(fid, '====================\n\n');
    fprintf(fid, 'Generated: %s\n\n', datestr(now));
    
    fprintf(fid, 'CONTROL GAINS:\n');
    fprintf(fid, 'Kp = %.3f * eye(6)\n', params.kp);
    fprintf(fid, 'Ki = %.3f * eye(6)\n\n', params.ki);
    
    fprintf(fid, 'TIMING PARAMETERS:\n');
    fprintf(fid, 'dt = %.3f s\n', params.dt);
    fprintf(fid, 'T_total = %.1f s\n\n', params.T_total);
    
    fprintf(fid, 'ROBOT PARAMETERS:\n');
    fprintf(fid, 'maxspeed = %.1f rad/s\n', params.maxspeed);
    
    fclose(fid);
end

function save_plots_as_images(fig, timestamp)
    % Save the current figure as high-resolution image
    filename = sprintf('results/simulation_plots_%s.png', timestamp);
    print(fig, filename, '-dpng', '-r300');
    
    % Also save as PDF for vector graphics
    filename_pdf = sprintf('results/simulation_plots_%s.pdf', timestamp);
    print(fig, filename_pdf, '-dpdf', '-bestfit');
end
