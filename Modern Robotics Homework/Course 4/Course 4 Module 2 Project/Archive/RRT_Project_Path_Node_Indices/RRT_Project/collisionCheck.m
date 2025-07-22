function collision = collisionCheck(pt1, pt2, obstacles)
collision = false;
for i = 1:size(obstacles,1)
    center = obstacles(i,1:2);
    radius = obstacles(i,3) / 2;  % Convert diameter to radius
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