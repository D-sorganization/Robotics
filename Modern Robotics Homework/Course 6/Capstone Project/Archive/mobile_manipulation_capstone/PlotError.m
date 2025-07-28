function PlotError(Xerr_log)
time = (0:size(Xerr_log,1)-1) * 0.01;
figure;
plot(time, Xerr_log);
xlabel('Time (s)');
ylabel('Error Twist Components');
legend('wx','wy','wz','vx','vy','vz');
title('End Effector Error Over Time');
grid on;
end