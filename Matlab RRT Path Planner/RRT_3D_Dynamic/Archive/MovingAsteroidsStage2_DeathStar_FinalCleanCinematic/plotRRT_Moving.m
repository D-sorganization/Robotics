function plotRRT_Moving(obstacles, path, nodes, start, goal, bounds)
figure('Color','k','Units','normalized','OuterPosition',[0 0 1 1]);
hold on; grid on; axis equal;
axis off;
xlim([bounds(1), bounds(2)]);
ylim([bounds(3), bounds(4)]);
zlim([bounds(5), bounds(6)]);
view(3);

% Load Falcon STL
fv = stlread('falcon_clean_fixed.stl');
original_vertices = fv.Points;
falcon_plot = patch('Faces',fv.ConnectivityList,'Vertices',original_vertices,'FaceColor',[0.8 0.8 0.8],'EdgeColor','none');

scale_factor = 0.1;

% Setup fixed camera
campos([-4 -4 3]);
camtarget([0 0 0]);
view([-45 25]);
camup([0 0 1]);
camva(30);

% Load velocities
load velocities.mat velocities;

% Obstacles plotting
obstacle_patches = gobjects(size(obstacles,1),1);
obstacle_types = strings(size(obstacles,1),1);
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
        obstacle_patches(i) = surf(xc, yc, zc, 'FaceColor', color, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
        obstacle_types(i) = "sphere";
    else
        [X,Y,Z] = ndgrid([-1 1]*size_param/2, [-1 1]*size_param/2, [-1 1]*size_param/2);
        vertices = [X(:), Y(:), Z(:)];
        faces = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
        patch_obj = patch('Vertices',vertices,'Faces',faces,'FaceColor',color,'FaceAlpha',0.2,'EdgeColor','none');
        patch_obj.Vertices = patch_obj.Vertices + [x y z];
        obstacle_patches(i) = patch_obj;
        obstacle_types(i) = "cube";
    end
end

% Animate moving asteroids
dt = 0.01;
t_original = linspace(0, 1, size(path,1));
t_smooth = linspace(0, 1, 250);
path_smooth = interp1(t_original, path, t_smooth, 'pchip');

v = VideoWriter('falcon_flight_moving_cleanfinal.mp4','MPEG-4');
open(v);

for frame = 1:size(path_smooth,1)-1
    p1 = path_smooth(frame,:);
    p2 = path_smooth(frame+1,:);
    dir = p2 - p1;
    angle = atan2(dir(2), dir(1));
    cosA = cos(angle);
    sinA = sin(angle);
    Rz = [cosA -sinA 0; sinA cosA 0; 0 0 1];

    roll_angle = deg2rad(5*randn);
    R_roll = [1 0 0; 0 cos(roll_angle) -sin(roll_angle); 0 sin(roll_angle) cos(roll_angle)];

    scaled_vertices = original_vertices * scale_factor;
    rotated_vertices = (scaled_vertices * Rz') * R_roll';
    translated_vertices = rotated_vertices + p1;
    falcon_plot.Vertices = translated_vertices;

    % Move and wrap obstacles
    for i = 1:size(obstacles,1)
        if obstacle_types(i) == "sphere"
            % Sphere move
            obstacle_patches(i).XData = obstacle_patches(i).XData + velocities(i,1)*dt;
            obstacle_patches(i).YData = obstacle_patches(i).YData + velocities(i,2)*dt;
            obstacle_patches(i).ZData = obstacle_patches(i).ZData + velocities(i,3)*dt;

            % Sphere wrap: recompute new center and redraw
            % Get current sphere center
            xc = mean(obstacle_patches(i).XData(:));
            yc = mean(obstacle_patches(i).YData(:));
            zc = mean(obstacle_patches(i).ZData(:));

            % Wrap center
            for d = 1:3
                low = bounds(d*2-1);
                high = bounds(d*2);
                range = high - low;
                if d == 1
                    xc = mod(xc - low, range) + low;
                elseif d == 2
                    yc = mod(yc - low, range) + low;
                else
                    zc = mod(zc - low, range) + low;
                end
            end

            r = obstacles(i,5);
            [theta,phi] = meshgrid(linspace(0,2*pi,20),linspace(0,pi,20));
            newX = xc + r*sin(phi).*cos(theta);
            newY = yc + r*sin(phi).*sin(theta);
            newZ = zc + r*cos(phi);
            obstacle_patches(i).XData = newX;
            obstacle_patches(i).YData = newY;
            obstacle_patches(i).ZData = newZ;
        else
            % Cube move
            obstacle_patches(i).Vertices = obstacle_patches(i).Vertices + velocities(i,:)*dt;

            % Cube wrap
            for d = 1:3
                low = bounds(d*2-1);
                high = bounds(d*2);
                range = high - low;
                shifted = obstacle_patches(i).Vertices(:,d) - low;
                obstacle_patches(i).Vertices(:,d) = mod(shifted, range) + low;
            end
        end
    end

    drawnow
    frame_captured = getframe(gcf);
    writeVideo(v, frame_captured);

    pause(0.0025);
end

close(v);
hold off;
end
