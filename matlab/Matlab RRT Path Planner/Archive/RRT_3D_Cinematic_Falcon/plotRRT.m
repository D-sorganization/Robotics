function plotRRT(obstacles, path, nodes, start, goal, bounds)
figure('Color','k'); % full black background
hold on; grid on; axis equal;
ax = gca;
ax.Color = 'k'; % axes background black
ax.XColor = 'w';
ax.YColor = 'w';
ax.ZColor = 'w';
ax.GridColor = 'w';
ax.BoxStyle = 'full'; % 3D box
xlabel('X','Color','w'); ylabel('Y','Color','w'); zlabel('Z','Color','w');
xlim([bounds(1), bounds(2)]);
ylim([bounds(3), bounds(4)]);
zlim([bounds(5), bounds(6)]);
view(3);

% Plot static starfield
num_stars = 500;
star_x = (bounds(2)-bounds(1))*rand(num_stars,1) + bounds(1);
star_y = (bounds(4)-bounds(3))*rand(num_stars,1) + bounds(3);
star_z = (bounds(6)-bounds(5))*rand(num_stars,1) + bounds(5);
plot3(star_x, star_y, star_z, 'w.', 'MarkerSize', 2);

% Plot obstacles
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
fv = stlread('falcon_clean_fixed.stl');
original_vertices = fv.Points;
falcon_plot = patch('Faces',fv.ConnectivityList,'Vertices',original_vertices,'FaceColor',[0.8 0.8 0.8],'EdgeColor','none');

scale_factor = 0.1;
trail = animatedline('Color','c','LineWidth',2); % blue exhaust trail

% Setup video writer
v = VideoWriter('falcon_flight.mp4','MPEG-4');
open(v);

% Animate Falcon along path
if ~isempty(path)
    for i = 1:size(path,1)-1
        p1 = path(i,:);
        p2 = path(i+1,:);
        dir = p2 - p1;
        angle = atan2(dir(2), dir(1));
        cosA = cos(angle);
        sinA = sin(angle);
        Rz = [cosA -sinA 0; sinA cosA 0; 0 0 1];

        scaled_vertices = original_vertices * scale_factor;
        rotated_vertices = scaled_vertices * Rz';
        translated_vertices = rotated_vertices + p1;
        falcon_plot.Vertices = translated_vertices;

        % Add to trail
        addpoints(trail, p1(1), p1(2), p1(3));

        % Camera follow
        campos(p1 + [-2 -2 1]);
        camtarget(p1);

        drawnow
        frame = getframe(gcf);
        writeVideo(v, frame);

        if i < size(path,1)-20
            pause(0.05); % normal speed
        else
            pause(0.005); % jump to lightspeed!
        end
    end
end

close(v);
title('3D Cinematic Falcon Space Flight','Color','w');
hold off;
end
