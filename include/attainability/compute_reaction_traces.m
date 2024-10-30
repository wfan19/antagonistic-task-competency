function [paths_af, paths_am] = compute_reaction_traces(ps, segment_twists, struct_design)
    N_ps = size(ps, 2);
    N_poses = size(segment_twists, 2);

    mat_rxns_3d = zeros(3, N_poses, N_ps);
    paths_af = zeros(N_poses, N_ps);
    paths_am= zeros(N_poses, N_ps);
    
    for i = 1 : N_ps
        % For each pressure, compute the corresponding reactions
        mat_rxns_3d(:, :, i) = calc_reaction_wrench(segment_twists, ps(:, i), struct_design);
    end
    
    [paths_af, paths_am] = mat_wrenches_to_traces(mat_rxns_3d);
end