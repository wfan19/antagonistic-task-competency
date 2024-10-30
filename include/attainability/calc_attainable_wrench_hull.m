function [bndry_af, bndry_am, cell_rltv_bndry_af, cell_rltv_bndry_am] = calc_attainable_wrench_hull(mat_segment_twists, struct_design)
    % Compute the absolute attainable wrench hulls
    boundary_ps = sample_edges_of_cuboid(3, struct_design.p_bounds);
    [bndry_af, bndry_am] = compute_reaction_traces(boundary_ps, mat_segment_twists, struct_design);

    i_bndry = boundary(bndry_af(1, :)', bndry_am(1, :)', 0);
    bndry_af = bndry_af(:, i_bndry);
    bndry_am = bndry_am(:, i_bndry);

    %% Compute the relative attainable wrench hulls.     
    if nargout > 2
        % Preallocate output
        N_nodes = size(bndry_af, 1);
        cell_rltv_bndry_af = cell(1, N_nodes);
        cell_rltv_bndry_am= cell(1, N_nodes);
    
        cell_rltv_bndry_af{1} = [0, 0];
        cell_rltv_bndry_am{1} = [0, 0];
    
        % Compute all the relative attainble reaction traces
        % These points will form the boundary of the relatively attainable
        % reaction space for each node. 
        rltv_bndry_af = bndry_af - bndry_af(1, :);
        rltv_bndry_am = bndry_am - bndry_am(1, :);
    
        for i = 2 : N_nodes
            % Order the points for each node, and only keep the ones that
            % contribute to the convex hull
            i_bndry = boundary(rltv_bndry_af(i, :)', rltv_bndry_am(i, :)', 0);
    
            cell_rltv_bndry_af{i} = rltv_bndry_af(i, i_bndry);
            cell_rltv_bndry_am{i} = rltv_bndry_am(i, i_bndry);
        end
    end
end