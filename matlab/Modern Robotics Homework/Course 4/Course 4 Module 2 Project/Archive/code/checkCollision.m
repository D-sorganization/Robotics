function collision = checkCollision(x1, y1, x2, y2, obstacles)
    collision = false;
    for i = 1:size(obstacles, 1)
        if collisionWithObstacle(x1, y1, x2, y2, obstacles(i, :))
            collision = true;
            return;
        end
    end
end
