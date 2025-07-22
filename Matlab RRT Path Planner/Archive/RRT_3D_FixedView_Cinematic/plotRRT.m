function plotRRT(obstacles, path, nodes, start, goal, bounds)
figure('Color','k');
hold on; grid on; axis equal;
ax = gca;
ax.Color = 'k';
ax.XColor = 'w';
ax.YColor = 'w';
ax.ZColor = 'w';
ax.GridColor = 'w';
ax.BoxStyle = 'full';
xlabel('X','Color','w'); ylabel('Y','Color','w'); zlabel('Z','Color','w');
xlim([bounds(1), bounds(2)]);
ylim([bounds(3), bounds(4)]);
zlim([bounds(5), bounds(6)]);
view(3);

% Starfield
num_stars = 500;
star_x = (bounds(2)-bounds(1))*rand(num_stars,1) + bounds(1);
star_y = (bounds(4)-bounds(3))*rand(num_stars,1) + bounds(3);
star_z = (bounds(6)-bounds(5))*rand(num_stars,1) + bounds(5);
plot3(star_x, star_y, star_z, 'w.', 'MarkerSize', 2);

% Obstacles
[theta,phi] = meshgrid(linspace(0,2*pi,20),linspace(0,pi,20));
for i = 1:size(obstacles,1)
    type = obstacles(i,1);
    x = obstacles(i,2);
    y = obstacles(i,3);
    z = obstacles(i,4);
    size_param = obstacles(i,5);
    color = obstacles(i,6:8);
    if type == 0
        r = size_param;
        xc = x + r*sin(phi).*cos(theta);
        yc = y + r*sin(phi).*sin(theta);
        zc = z + r*cos(phi);
        surf(xc, yc, zc, 'FaceColor', color, 'FaceAlpha', 0.6, 'EdgeColor', 'none');
    else
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

% Animate tree (skip for speed)

% Load Falcon STL
fv = stlread('falcon_clean_fixed.stl');
original_vertices = fv.Points;
falcon_plot = patch('Faces',fv.ConnectivityList,'Vertices',original_vertices,'FaceColor',[0.8 0.8 0.8],'EdgeColor','none');

scale_factor = 0.5; % Make Falcon big

% Setup fixed camera
campos([0 -5 1]);
camtarget([0 0 0]);
camup([0 0 1]);
camva(30); % narrow view angle for cinematic zoom

% Plot full path trace
plot3(path(:,1), path(:,2), path(:,3), 'c-', 'LineWidth', 2);

% Smooth interpolation
t_original = linspace(0, 1, size(path,1));
t_smooth = linspace(0, 1, 1000); % fine smooth
path_smooth = interp1(t_original, path, t_smooth, 'pchip');

% Setup video
v = VideoWriter('falcon_flight.mp4','MPEG-4');
open(v);

% Animate Falcon smoothly
for i = 1:size(path_smooth,1)-1
    p1 = path_smooth(i,:);
    p2 = path_smooth(i+1,:);
    dir = p2 - p1;
    angle = atan2(dir(2), dir(1));
    cosA = cos(angle);
    sinA = sin(angle);
    Rz = [cosA -sinA 0; sinA cosA 0; 0 0 1];

    scaled_vertices = original_vertices * scale_factor;
    rotated_vertices = scaled_vertices * Rz';
    translated_vertices = rotated_vertices + p1;
    falcon_plot.Vertices = translated_vertices;

    drawnow
    frame = getframe(gcf);
    writeVideo(v, frame);

    pause(0.01); % 10 sec total
end

close(v);
title('3D Cinematic Fixed View Falcon Flight','Color','w');
hold off;
end
