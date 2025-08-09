function plotRRT(obstacles, path, nodes, start, goal, bounds)
figure('Color','k'); % black background
hold on; grid on; axis equal;
ax = gca;
ax.XColor = 'w';
ax.YColor = 'w';
ax.ZColor = 'w';
ax.GridColor = 'w';
xlabel('X','Color','w'); ylabel('Y','Color','w'); zlabel('Z','Color','w');
xlim([bounds(1), bounds(2)]);
ylim([bounds(3), bounds(4)]);
zlim([bounds(5), bounds(6)]);
view(3);

% Plot obstacles first
[theta,phi] = meshgrid(linspace(0,2*pi,20),linspace(0,pi,20));
for i = 1:size(obstacles,1)
    type = obstacles(i,1);
    x = obstacles(i,2);
    y = obstacles(i,3);
    z = obstacles(i,4);
    size_param = obstacles(i,5);
    color = obstacles(i,6:8);
    if type == 0  % Sphere
        r = size_param;
        xc = x + r*sin(phi).*cos(theta);
        yc = y + r*sin(phi).*sin(theta);
        zc = z + r*cos(phi);
        surf(xc, yc, zc, 'FaceColor', color, 'FaceAlpha', 0.6, 'EdgeColor', 'none');
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
        patch('Vertices',[cubeX+x cubeY+y cubeZ+z],'Faces',faces,'FaceColor',color,'FaceAlpha',0.6,'EdgeColor','none');
    end
end

% Animate tree growing
for i = 2:size(nodes,1)
    parent = nodes(i,4);
    plot3([nodes(i,1) nodes(parent,1)], [nodes(i,2) nodes(parent,2)], [nodes(i,3) nodes(parent,3)], 'b');
    drawnow limitrate
end

% Plot start and goal
plot3(start(1), start(2), start(3), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);

% Load Falcon STL
[fv.vertices, fv.faces] = stlread('falcon_simple.stl');
falcon_plot = patch('Faces',fv.faces,'Vertices',fv.vertices,'FaceColor',[0.8 0.8 0.8],'EdgeColor','none');

% Scale Falcon smaller
scale_factor = 0.1;
scaled_vertices = fv.vertices * scale_factor;

% Animate Falcon along path
if ~isempty(path)
    for i = 1:size(path,1)-1
        p1 = path(i,:);
        p2 = path(i+1,:);
        dir = p2 - p1;
        angle = atan2(dir(2), dir(1));  % yaw angle
        R = makehgtform('zrotate', angle);
        T = makehgtform('translate', p1);
        rotated_vertices = (R(1:3,1:3) * scaled_vertices')';
        translated_vertices = rotated_vertices + p1;
        falcon_plot.Vertices = translated_vertices;
        drawnow
        pause(0.05);
    end
end

title('3D RRT Path Planning with Falcon Animation','Color','w');
hold off;
end

function [V,F] = stlread(filename)
    [F,V] = deal([]);
    fid = fopen(filename,'r');
    if fid == -1
        error('File could not be opened, check name or path.')
    end
    tline = fgetl(fid);
    while ischar(tline)
        if contains(tline,'vertex')
            temp = sscanf(tline,'%*s %f %f %f');
            V = [V; temp'];
        elseif contains(tline,'endfacet')
            F = [F; size(V,1)-2 size(V,1)-1 size(V,1)];
        end
        tline = fgetl(fid);
    end
    fclose(fid);
end
