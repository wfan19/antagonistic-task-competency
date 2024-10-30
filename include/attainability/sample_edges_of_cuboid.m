function points_out = sample_edges_of_cuboid(N_points_per_edge, scales)
    N_dims = length(scales);

    s_along_edge = linspace(0, 1, N_points_per_edge);

    vertices = ff2n(N_dims)';
    
    points = [];
    for i_vert = 1 : size(vertices, 2)
        % Add points on each of the edges connected to this vertex
        vert_i = vertices(:, i_vert);
        for j_dim = 1 : N_dims
            vert_i_repeat = repmat(vert_i, [1, N_points_per_edge]);
            vert_i_repeat(j_dim, :) = s_along_edge;

            points = [points, vert_i_repeat];
        end
    end
    points = unique(points', 'rows')';

    points_out = diag(scales) * points;
end