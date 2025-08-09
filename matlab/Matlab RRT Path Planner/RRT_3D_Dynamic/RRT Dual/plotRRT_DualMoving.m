function plotRRT_DualMoving(obstacles, bounds)
% plotRRT_DualMoving.m
% Falcon chasing TIE Fighter using dual RRTs

% --- Figure Setup ---
figure('Color','k','Units','normalized','OuterPosition',[0 0 1 1]);
hold on; grid on; axis equal;
axis off;
xlim([bounds(1), bounds(2)]);
ylim([bounds(3), bounds(4)]);
zlim([bounds(5), bounds(6)]);
view(3);
camva(8);
set(gca,'Color','k');

% --- Load Models ---
falcon_stl = stlread('falcon_clean_fixed.stl');

% Placeholder TIE Fighter (gray cube)
tie_fighter_vertices = 0.05 * [
    -1 -1 -1;
    -1 -1  1;
    -1  1 -1;
    -1  1  1;
     1 -1 -1;
     1 -1  1;
     1  1 -1;
     1  1  1
];
tie_fighter_faces = [
    1 2 4 3;
    5 6 8 7;
    1 2 6 5;
    3 4 8 7;
    1 3 7 5;
    2 4 8 6
];

tie_patch = patch('Faces', tie_fighter_faces, 'Vertices', zeros(8,3), ...
                  'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'none');
falcon_patch = patch('Faces', falcon_stl.ConnectivityList, 'Vertices', zeros(size(falcon_stl.Points)), ...
                     'FaceColor', [1 1 0], 'EdgeColor', 'none'); % Falcon gold for now

% --- Initial Conditions ---
start_TIE = [-0.8, -0.5, 0];
goal_TIE  = [0.8, 0.5, 0];
start_Falcon = [-0.8, 0.5, 0];

% --- Precompute TIE Fighter Path ---
[nodes_TIE, path_TIE] = RRT(start_TIE, goal_TIE, obstacles, bounds);

% --- Initialize Variables ---
tie_idx = 1;
falcon_pos = start_Falcon;

% --- Obstacles Movement Setup (Optional) ---
dt = 0.02;
num_frames = 300;

% --- Animation Loop ---
for frame = 1:num_frames
    % --- Move TIE Fighter ---
    if tie_idx < size(path_TIE,1)
        tie_pos = path_TIE(tie_idx,:);
        tie_idx = tie_idx + 1;
    else
        tie_pos = goal_TIE;
    end
    tie_patch.Vertices = tie_fighter_vertices + tie_pos;
    
    % --- Falcon Plans to Chase TIE ---
    [nodes_Falcon, path_Falcon] = RRT(falcon_pos, tie_pos, obstacles, bounds);
    
    % --- Move Falcon One Step ---
    if size(path_Falcon,1) > 1
        falcon_pos = path_Falcon(2,:); % Move to next step
    end
    
    scaled_falcon = 0.08 * falcon_stl.Points; % Scale Falcon small
    falcon_patch.Vertices = scaled_falcon + falcon_pos;
    
    % --- Draw Obstacles (Optional: move if you want) ---
    
    drawnow;
    pause(0.01); % Frame control
end

hold off;
end
