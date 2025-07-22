function plotRRT_Moving(obstacles, path, nodes, start, goal, bounds)
% plotRRT_Moving.m (Version 3 - STL Model & Fixed Camera)
% Generates a cinematic animation using an STL model for the spaceship
% and a fixed third-person camera view.

%% --- Setup Figure and FIXED Camera ---
fig = figure('Color','k', 'Units','pixels', 'Position', [100 100 1600 900]);
hold on; axis off;
set(gca, 'Color', 'k', 'DataAspectRatio', [1 1 1]);
set(gca, 'Units', 'normalized', 'Position', [0 0 1 1]);
xlim(bounds(1:2)); ylim(bounds(3:4)); zlim(bounds(5:6));

% Set a fixed third-person viewpoint.
% The camera will not move during the animation.
campos([-1.5, -1.5, 1.0]); % Position the camera behind and above the start
camtarget([0, 0, 0]);      % Aim the camera at the center of the scene
camup([0, 0, 1]);          % Ensure the Z-axis is "up"
camva(30);

%% --- Plot Static Obstacles ---
obstacle_patches = gobjects(size(obstacles,1),1);
for i = 1:size(obstacles,1)
    type = obstacles(i,1);
    center = obstacles(i,2:4);
    size_param = obstacles(i,5);
    color = obstacles(i,6:8);
    if type == 0 % Sphere
        [x,y,z] = sphere(20);
        obstacle_patches(i) = surf(x*size_param+center(1), y*size_param+center(2), z*size_param+center(3), ...
            'FaceColor', color, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    else % Cube
        verts = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1] * size_param / 2;
        faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 4 3 7 8; 1 4 8 5; 2 3 7 6];
        obstacle_patches(i) = patch('Vertices', verts + center, 'Faces', faces, ...
            'FaceColor', color, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    end
end
light('Position',[5 10 5], 'Style','infinite');

%% --- Define the Spaceship Model (from STL file) ---
disp('Loading STL model...');
% 1. Load the model using stlread
falcon_model = stlread('falcon_clean_fixed.stl');
falcon_verts_base = falcon_model.Points;
falcon_faces = falcon_model.ConnectivityList;

% 2. Center the model at the origin to ensure correct rotation
vertex_mean = mean(falcon_verts_base, 1);
falcon_verts_base = falcon_verts_base - vertex_mean;

% 3. Scale the model to an appropriate size for the scene
% (You can adjust this scale factor if the ship is too big or small)
falcon_scale = 0.0015;
falcon_verts_base = falcon_verts_base * falcon_scale;

% 4. Create the patch object for the Falcon before the loop
falcon_patch = patch('Faces', falcon_faces, 'Vertices', falcon_verts_base, ...
                     'FaceColor', [0.75, 0.75, 0.8], 'EdgeColor', 'none', ...
                     'FaceLighting', 'gouraud', 'AmbientStrength', 0.4);

%% --- Smooth Path for Animation ---
t_original = linspace(0, 1, size(path,1));
t_smooth = linspace(0, 1, 500); % 500 frames for the animation
path_smooth = interp1(t_original, path, t_smooth, 'pchip');

%% --- Main Animation Loop ---
v = VideoWriter('falcon_flight_fixed_view.mp4', 'MPEG-4');
v.FrameRate = 30;
open(v);

disp('🎬 Generating animation frames from fixed viewpoint...');

for i = 1:size(path_smooth,1)-1
    p1 = path_smooth(i,:);
    p2 = path_smooth(i+1,:);
    
    % --- Calculate Spaceship Orientation ---
    forward = (p2 - p1);
    if norm(forward) < 1e-6; continue; end
    forward = forward / norm(forward);
    world_up = [0 0 1];
    right = cross(world_up, forward);
    if norm(right) < 1e-6; right = cross([1 0 0], forward); end
    right = right / norm(right);
    up = cross(forward, right);
    R_falcon = [forward', right', up'];

    % --- APPLY ROTATION AND TRANSLATION TO THE MODEL ---
    rotated_verts = (R_falcon * falcon_verts_base')';
    translated_verts = rotated_verts + p1;
    set(falcon_patch, 'Vertices', translated_verts);

    % --- Capture Frame ---
    % Note: The camera position is NOT updated in the loop anymore.
    drawnow;
    frame_img = getframe(fig);
    writeVideo(v, frame_img);
end

close(v);
disp('✅ Video saved as falcon_flight_fixed_view.mp4');

end