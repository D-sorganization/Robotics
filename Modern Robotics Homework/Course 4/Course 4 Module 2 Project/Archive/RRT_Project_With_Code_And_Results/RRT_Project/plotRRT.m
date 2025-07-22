function plotRRT(obstacles, path, nodes, start, goal)
figure; hold on; axis equal;
xlim([-0.6 0.6]); ylim([-0.6 0.6]);
theta = linspace(0,2*pi,50);
robot_radius = 0.02;

% Plot obstacles
for i = 1:size(obstacles,1)
    xc = obstacles(i,1); yc = obstacles(i,2); r = obstacles(i,3) / 2;
    fill(r*cos(theta)+xc, r*sin(theta)+yc, 'k');
end

% Plot RRT tree
for i = 2:size(nodes,1)
    plot([nodes(i,1) nodes(nodes(i,3),1)], [nodes(i,2) nodes(nodes(i,3),2)], 'b-');
end

% Overlay robot radius at each path point
if ~isempty(path)
    for i = 1:size(path,1)
        fill(robot_radius*cos(theta)+path(i,1), robot_radius*sin(theta)+path(i,2), ...
             'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    end
    plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);
end

% Plot start and goal
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

title('RRT Path Planning with Robot Radius Overlay');
xlabel('X'); ylabel('Y');
hold off;
end
