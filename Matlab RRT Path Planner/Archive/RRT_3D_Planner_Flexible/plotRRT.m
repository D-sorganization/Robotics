function plotRRT(obstacles, path, nodes, start, goal)
figure;
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-0.6 0.6]); ylim([-0.6 0.6]); zlim([-0.6 0.6]);
view(3);

% Plot obstacles
[theta,phi] = meshgrid(linspace(0,2*pi,20),linspace(0,pi,20));
for i = 1:size(obstacles,1)
    type = obstacles(i,1);
    x = obstacles(i,2);
    y = obstacles(i,3);
    z = obstacles(i,4);
    size_param = obstacles(i,5);
    if type == 0  % Sphere
        r = size_param;
        xc = x + r*sin(phi).*cos(theta);
        yc = y + r*sin(phi).*sin(theta);
        zc = z + r*cos(phi);
        surf(xc, yc, zc, 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    else  % Cube
        [X,Y,Z] = ndgrid([-1 1]*size_param/2, [-1 1]*size_param/2, [-1 1]*size_param/2);
        cubeX = X(:); cubeY = Y(:); cubeZ = Z(:);
        faces = [
            1 2 4 3;
            5 6 8 7;
            1 2 6 5;
            3 4 8 7;
            1 3 7 5;
            2 4 8 6
        ];
        patch('Vertices',[cubeX+x cubeY+y cubeZ+z],'Faces',faces,'FaceColor',[0.7 0.7 0.7],'FaceAlpha',0.3,'EdgeColor','none');
    end
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

title('3D RRT Path Planning with Spheres and Cubes');
hold off;
end
