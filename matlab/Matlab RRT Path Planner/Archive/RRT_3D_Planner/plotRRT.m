function plotRRT(obstacles, path, nodes, start, goal)
figure;
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-0.6 0.6]); ylim([-0.6 0.6]); zlim([-0.6 0.6]);
view(3);

% Plot obstacles
[theta,phi] = meshgrid(linspace(0,2*pi,20),linspace(0,pi,20));
for i = 1:size(obstacles,1)
    r = obstacles(i,4);
    xc = obstacles(i,1) + r*sin(phi).*cos(theta);
    yc = obstacles(i,2) + r*sin(phi).*sin(theta);
    zc = obstacles(i,3) + r*cos(phi);
    surf(xc, yc, zc, 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end

% Plot tree
for i = 2:size(nodes,1)
    parent = nodes(i,4);
    plot3([nodes(i,1) nodes(parent,1)], [nodes(i,2) nodes(parent,2)], [nodes(i,3) nodes(parent,3)], 'b');
end

% Plot path
if ~isempty(path)
    plot3(path(:,1), path(:,2), path(:,3), 'r-', 'LineWidth', 2);
end

% Plot start and goal
plot3(start(1), start(2), start(3), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);

title('3D RRT Path Planning');
hold off;
end
