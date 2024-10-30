function s = arclength_parameterize_convex_shape(pts)
        N_points = size(pts, 2);
        s = zeros(1, N_points);
        for i = 2 : N_points
            ds = norm(pts(:, i) - pts(:, i-1));

            s(i) = s(i-1) + ds;
        end

        s = s / s(end);
end

