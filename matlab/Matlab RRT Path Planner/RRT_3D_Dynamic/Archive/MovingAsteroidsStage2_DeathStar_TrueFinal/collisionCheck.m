function collision = collisionCheck(pt1, pt2, obstacles)
collision = false;
for i = 1:size(obstacles,1)
    if size(obstacles,2) == 4
        % Old format: spheres only
        type = 0;
        center = obstacles(i,1:3);
        size_param = obstacles(i,4);
    else
        % New format: mixed spheres and cubes
        type = obstacles(i,1);
        center = obstacles(i,2:4);
        size_param = obstacles(i,5);
    end

    if type == 0  % Sphere
        v = pt2 - pt1;
        u = center - pt1;
        proj = dot(u,v) / norm(v)^2;
        proj = max(0, min(1, proj));
        closest = pt1 + proj * v;
        if norm(closest - center) <= size_param
            collision = true;
            return;
        end
    else  % Cube
        min_corner = center - size_param;
        max_corner = center + size_param;
        for t = linspace(0, 1, 20)
            p = (1-t)*pt1 + t*pt2;
            if all(p >= min_corner) && all(p <= max_corner)
                collision = true;
                return;
            end
        end
    end
end
end
