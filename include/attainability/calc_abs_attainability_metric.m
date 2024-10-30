function metric_out = calc_abs_attainability_metric(segment_twists, w_tip, struct_design, K)
    arguments
        segment_twists
        w_tip
        struct_design
        K = [1, 1];
    end

    N_nodes = size(segment_twists, 2);
    
    [cell_bndry_af, cell_bndry_am] = calc_attainable_wrench_hull_complete(segment_twists, struct_design);
    a_requirements = -calc_external_wrench(segment_twists, w_tip, struct_design.g_0);

    metric_out = 0;
    for i = 1 : N_nodes
        % Compute the minimum distance from each requirement point to
        % absolute attainable wrench space sets
        requirement_i = a_requirements([1, 3], i);
        bndry_verts_i = [cell_bndry_af{i}; cell_bndry_am{i}];
        metric_out = metric_out + distance_to_convex_set(requirement_i, bndry_verts_i, K);
    end
end

