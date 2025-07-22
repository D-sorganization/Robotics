function plotRRT(obstacles, path, nodes, start, goal)
figure; hold on; axis equal;
xlim([-1 1]); ylim([-1 1]);
theta = linspace(0,2*pi,50);
for i = 1:size(obstacles,1)
    xc = obstacles(i,1); yc = obstacles(i,2); r = obstacles(i,3);
    fill(r*cos(theta)+xc, r*sin(theta)+yc, 'k');
end
for i = 2:size(nodes,1)
    plot([nodes(i,1) nodes(nodes(i,3),1)], [nodes(i,2) nodes(nodes(i,3),2)], 'b-');
end
if ~isempty(path)
    plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);
end
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
title('RRT Path Planning');
xlabel('X');
ylabel('Y');
hold off;
end