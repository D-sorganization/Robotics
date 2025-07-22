function collision = collisionCheck(pt1, pt2, obstacles)
% Define the radius of the robot (e.g., Kilobot ~ 0.017 m, use 0.02 conservatively)
r_robot = 0.02;

collision = false;
for i = 1:size(obstacles,1)
    center = obstacles(i,1:2);
    radius = (obstacles(i,3) / 2) + r_robot;  % Obstacle radius + robot radius

    v = pt2 - pt1;
    u = center - pt1;
    proj = dot(u,v) / norm(v)^2;
    proj = max(0, min(1, proj));
    closest_point = pt1 + proj * v;

    if norm(closest_point - center) <= radius
        collision = true;
        return;
    end
end
end