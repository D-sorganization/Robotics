function is_on = isPointOnSegment(pt, x1, y1, x2, y2)
    % Allow tiny numerical tolerance
    epsilon = 1e-6;
    min_x = min(x1, x2) - epsilon;
    max_x = max(x1, x2) + epsilon;
    min_y = min(y1, y2) - epsilon;
    max_y = max(y1, y2) + epsilon;
    is_on = pt(1) >= min_x && pt(1) <= max_x && pt(2) >= min_y && pt(2) <= max_y;
end
