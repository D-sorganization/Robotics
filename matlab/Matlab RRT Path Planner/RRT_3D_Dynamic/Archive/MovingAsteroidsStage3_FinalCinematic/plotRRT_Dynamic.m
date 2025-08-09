function plotRRT_Dynamic(obstacles, path, nodes, start, goal, bounds)
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
campos([0 -5 1]);
camtarget([0 0 0]);
camup([0 0 1]);
camva(30);

% Random velocities (loaded separately)
load velocities.mat velocities;

% Obstacles initial plotting
obstacle_patches = gobjects(size(obstacles,1),1);
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
    else
        [X,Y,Z] = ndgrid([-1 1]*size_param/2, [-1 1]*size_param/2, [-1 1]*size_param/2);
        vertices = [X(:), Y(:), Z(:)];
        faces = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
        patch_obj = patch('Vertices',vertices,'Faces',faces,'FaceColor',color,'FaceAlpha',0.2,'EdgeColor','none');
        patch_obj.Vertices = patch_obj.Vertices + [x y z];
        obstacle_patches(i) = patch_obj;
    end
end

% Animate Falcon dynamically
dt = 0.01;
t_original = linspace(0, 1, size(path,1));
t_smooth = linspace(0, 1, 250);
path_smooth = interp1(t_original, path, t_smooth, 'pchip');

v = VideoWriter('falcon_flight_dynamic.mp4','MPEG-4');
open(v);

current_idx = 1;
goal_reached = false;

while ~goal_reached && current_idx < size(path_smooth,1)-1
    p1 = path_smooth(current_idx,:);
    p2 = path_smooth(current_idx+1,:);
    
    % Check collision ahead
    collision = false;
    interp_steps = 5;
    for interp = linspace(0, 1, interp_steps)
        check_point = (1-interp)*p1 + interp*p2;
        if collisionCheckDynamic(check_point, obstacles, obstacle_patches)
            collision = true;
            break;
        end
    end
    
    if collision
        % Replan
        disp('Collision detected! Replanning...');
        [new_path, ~] = RRT(p1, goal, obstacles, bounds, 0.1, 2000, 0.05);
        t_original_new = linspace(0, 1, size(new_path,1));
        path_smooth = interp1(t_original_new, new_path, linspace(0, 1, 250));
        current_idx = 1;
        continue;
    end
    
    % Move Falcon
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

    % Move obstacles
    for i = 1:size(obstacles,1)
        obstacle_patches(i).Vertices = obstacle_patches(i).Vertices + velocities(i,:) * dt;
        for d = 1:3
            low = bounds(d*2-1);
            high = bounds(d*2);
            range = high - low;
            shifted = obstacle_patches(i).Vertices(:,d) - low;
            obstacle_patches(i).Vertices(:,d) = mod(shifted, range) + low;
        end
    end

    drawnow
    frame_captured = getframe(gcf);
    writeVideo(v, frame_captured);

    pause(0.0025);
    current_idx = current_idx + 1;
    
    % Check goal
    if norm(p2 - goal) < 0.1
        goal_reached = true;
    end
end

close(v);
disp('Goal reached!');
hold off;
end
