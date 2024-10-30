function [cell_bndry_af, cell_bndry_am] = calc_attainable_wrench_hull_complete(mat_segment_twists, struct_design)
    boundary_ps = sample_edges_of_cuboid(3, struct_design.p_bounds);
    [bndry_af, bndry_am] = compute_reaction_traces(boundary_ps, mat_segment_twists, struct_design);

    [cell_bndry_af, cell_bndry_am] = extract_attainable_wrench_hull_complete(bndry_af, bndry_am);

end

