function plotError(Xerr_log, filename)
% plotError - Plot the error evolution over time
% Inputs:
%   Xerr_log - Nx6 matrix of error values over time
%   filename - Output plot filename (optional)

    N = size(Xerr_log, 1);
    dt = 0.01;  % 10ms timestep
    time = (0:N-1) * dt;
    
    figure('Position', [100, 100, 800, 600]);
    
    % Plot all 6 error components
    subplot(2,1,1);
    plot(time, Xerr_log(:,1:3));
    xlabel('Time (s)');
    ylabel('Angular Error (rad)');
    title('End-Effector Angular Error');
    legend('\omega_x', '\omega_y', '\omega_z', 'Location', 'best');
    grid on;
    
    subplot(2,1,2);
    plot(time, Xerr_log(:,4:6));
    xlabel('Time (s)');
    ylabel('Linear Error (m)');
    title('End-Effector Linear Error');
    legend('v_x', 'v_y', 'v_z', 'Location', 'best');
    grid on;
    
    % Save if filename provided
    if nargin > 1
        saveas(gcf, filename);
        fprintf('Error plot saved: %s\n', filename);
    end
end