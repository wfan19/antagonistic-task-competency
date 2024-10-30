function ax = plot_wrench_hull_white_lie(bndry_af, bndry_am, ax, color, plot_style, args)

    arguments
        bndry_af
        bndry_am
        ax = axes(figure());
        color = [255 48 150] / 255;
        plot_style = struct("facealpha", 0.6, "linewidth", 1)
        args.debug = false
    end

    N_nodes = size(bndry_af, 1);

    [cell_bndry_af, cell_bndry_am] = extract_attainable_wrench_hull_complete(bndry_af, bndry_am);

    i_bndry_at_base = boundary(bndry_af(1, :)', bndry_am(1, :)', 0);
    af_vert_original = bndry_af(:, i_bndry_at_base);
    am_vert_original = bndry_am(:, i_bndry_at_base);

    af_vert_on_bndry = zeros(size(af_vert_original));
    am_vert_on_bndry = zeros(size(am_vert_original));
    % For each layer, find the closest point on the boundary to the point
    % from the trace that was on the boundary at the base
    for i_node = 1 : N_nodes
        if i_node == 1
            af_vert_on_bndry(1, :) = af_vert_original(1, :);
            am_vert_on_bndry(1, :) = am_vert_original(1, :);
        else
            original_pts_i = [af_vert_original(i_node, :); am_vert_original(i_node, :)];
            verts_i = [cell_bndry_af{i_node}; cell_bndry_am{i_node}];
            bndry_pts_i = find_nearest_points_on_convex_boundary(verts_i, original_pts_i);

            af_vert_on_bndry(i_node, :) = bndry_pts_i(1, :);
            am_vert_on_bndry(i_node, :) = bndry_pts_i(2, :);
        end
    end

    ax = plot_wrench_hull(af_vert_on_bndry, am_vert_on_bndry, ax, color, plot_style);
end

