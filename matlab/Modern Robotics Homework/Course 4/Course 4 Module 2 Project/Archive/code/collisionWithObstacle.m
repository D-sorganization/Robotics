function collision = collisionWithObstacle(x1, y1, x2, y2, obstacle)
    [intersects, points] = lineCircleIntersections(x1, y1, x2, y2, obstacle);
    collision = any(arrayfun(@(i) isPointOnSegment(points(i,:), x1, y1, x2, y2), 1:size(points,1)));
end
