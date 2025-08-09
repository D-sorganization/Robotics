function [intersects, points] = lineCircleIntersections(x1, y1, x2, y2, obstacle)
    x_c = obstacle(1);
    y_c = obstacle(2);
    r = obstacle(3);

    dx = x2 - x1;
    dy = y2 - y1;
    a = dx^2 + dy^2;
    b = 2 * (dx * (x1 - x_c) + dy * (y1 - y_c));
    c = (x1 - x_c)^2 + (y1 - y_c)^2 - r^2;

    discriminant = b^2 - 4 * a * c;
    points = [];

    if discriminant < 0
        intersects = false;
        return;
    end

    sqrt_disc = sqrt(discriminant);
    t1 = (-b + sqrt_disc) / (2 * a);
    t2 = (-b - sqrt_disc) / (2 * a);

    points = [x1 + t1 * dx, y1 + t1 * dy; x1 + t2 * dx, y1 + t2 * dy];
    intersects = true;
end
