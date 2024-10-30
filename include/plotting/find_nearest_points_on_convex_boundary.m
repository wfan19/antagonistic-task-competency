function closest_bndry_pts = find_nearest_points_on_convex_boundary(vertices, query_pts)

    s_verts = arclength_parameterize_convex_shape(vertices);
    
    N_sample = 300;
    s_sample = linspace(0, 1, N_sample);
    % Sample a lot of points along the boundary of the convex shape
    x_sample = interp1(s_verts, vertices(1, :), s_sample);
    y_sample = interp1(s_verts, vertices(2, :), s_sample);
    pts_sample = [x_sample; y_sample];

    N_pts = size(query_pts, 2);
    closest_bndry_pts = zeros(size(query_pts));
    for i = 1 : N_pts
        p_i = query_pts(:, i);
        dists = vecnorm(p_i - pts_sample, 2, 1);
        [~, i_min] = min(dists);
        
        closest_bndry_pts(:, i) = pts_sample(:, i_min);
    end

end

