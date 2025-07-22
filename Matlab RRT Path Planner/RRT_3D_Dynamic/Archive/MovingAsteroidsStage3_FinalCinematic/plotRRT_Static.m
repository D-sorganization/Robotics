function plotRRT_Static(obstacles, path, nodes, start, goal, bounds)
figure('Color','w','Units','normalized','OuterPosition',[0 0 1 1]);
hold on; grid on; axis equal;
xlabel('X','Color','k'); ylabel('Y','Color','k'); zlabel('Z','Color','k');
xlim([bounds(1), bounds(2)]);
ylim([bounds(3), bounds(4)]);
zlim([bounds(5), bounds(6)]);
view(3);

% Obstacles (gray and transparent)
[theta,phi] = meshgrid(linspace(0,2*pi,20),linspace(0,pi,20));
for i = 1:size(obstacles,1)
    type = obstacles(i,1);
    x = obstacles(i,2);
    y = obstacles(i,3);
    z = obstacles(i,4);
    size_param = obstacles(i,5);
    if type == 0
        % Sphere
        r = size_param;
        xc = x + r*sin(phi).*cos(theta);
        yc = y + r*sin(phi).*sin(theta);
        zc = z + r*cos(phi);
        surf(xc, yc, zc, 'FaceColor', [0.7 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    else
        % Cube
        [X,Y,Z] = ndgrid([-1 1]*size_param/2, [-1 1]*size_param/2, [-1 1]*size_param/2);
        vertices = [X(:), Y(:), Z(:)];
        
        % Random rotation
        angles = rand(1,3) * 2 * pi;
        Rx = [1 0 0; 0 cos(angles(1)) -sin(angles(1)); 0 sin(angles(1)) cos(angles(1))];
        Ry = [cos(angles(2)) 0 sin(angles(2)); 0 1 0; -sin(angles(2)) 0 cos(angles(2))];
        Rz = [cos(angles(3)) -sin(angles(3)) 0; sin(angles(3)) cos(angles(3)) 0; 0 0 1];
        R = Rz * Ry * Rx;
        
        rotated_vertices = (R * vertices')';
        rotated_vertices(:,1) = rotated_vertices(:,1) + x;
        rotated_vertices(:,2) = rotated_vertices(:,2) + y;
        rotated_vertices(:,3) = rotated_vertices(:,3) + z;
        
        faces = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
        patch('Vertices',rotated_vertices,'Faces',faces,'FaceColor',[0.7 0.7 0.7],'FaceAlpha',0.3,'EdgeColor','none');
    end
end

% Plot RRT edges
for i = 2:size(nodes,1)
    parent = nodes(i,4);
    plot3([nodes(i,1) nodes(parent,1)], [nodes(i,2) nodes(parent,2)], [nodes(i,3) nodes(parent,3)], 'Color',[0.7 0.7 0.7]);
end

% Plot final smooth path
plot3(path(:,1), path(:,2), path(:,3), 'r-', 'LineWidth', 2);

% Start and goal markers
plot3(start(1), start(2), start(3), 'go', 'MarkerFaceColor','g', 'MarkerSize',8);
plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerFaceColor','r', 'MarkerSize',8);

title('Static RRT Visualization with Rotated Cubes','Color','k');

% Save static figure
saveas(gcf, 'rrt_static_view.png');
hold off;
end
