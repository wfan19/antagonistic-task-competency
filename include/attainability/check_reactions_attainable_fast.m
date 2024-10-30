function attainable = check_reactions_attainable_fast(reaction_requirement, segment_twists, struct_design, p_bounds)
    
    N_nodes = size(segment_twists, 2);
    f_required = reaction_requirement(1, :);
    m_required = reaction_requirement(3, :);

    %%% Create the boundary reactions
    % Generate pressures along the edges of pressure space
    N_ps_per_edge = 2;
    ps_bndry = sample_edges_of_cuboid(2, p_bounds);
    N_ps = size(ps_bndry, 2);

    % Compute the corresponding reactions these pressures induce, for the
    % given arm design and segment twists
    reactions_bndry = zeros(3, N_nodes, N_ps);
    for i = 1 : N_ps
        ps_i = ps_bndry(:, i);
        reactions_bndry(:, :, i) = calc_reaction_wrench(segment_twists, ps_i, struct_design);
    end
    [traces_f_bndry, traces_m_bndry] = mat_wrenches_to_traces(reactions_bndry);
    i_contour = boundary(traces_f_bndry(1, :)', traces_m_bndry(1, :)', 0.1);
    rltv_f_bndry = traces_f_bndry - traces_f_bndry(1, :);
    rltv_m_bndry = traces_m_bndry - traces_m_bndry(1, :);
    
    %%% Test the attainability of the reaction
    % Test 1: are the reactions themselves bounded by the reactions from
    % the boundary pressures
    v_rxn_in_bndry = false(N_nodes, 1);
    for i = 1 : N_nodes
        v_rxn_in_bndry(i) = inpolygon(f_required(i), m_required(i), traces_f_bndry(i, i_contour), traces_m_bndry(i, i_contour));
    end
    attainable = all(v_rxn_in_bndry);
    if ~attainable
        return;
    end

    % Test 2: are the changes in requierd reaction along the arm bounded by
    % the changes in of boundary reactions along the arm?
    % TODO: Better name for the concept of " change in required reaction"
    % Maybe "relative (rltv) reaction"?
    rltv_reaction_requirement = reaction_requirement - reaction_requirement(:, 1);
    rltv_f = rltv_reaction_requirement(1, :);
    rltv_m = rltv_reaction_requirement(3, :);

    % Fast check: are all reactions within the convex hull of the boundary
    % pressures? If all no then return no. If yes, then proceed to the
    % check for each s
    i_contour_all = boundary(rltv_f_bndry(:), rltv_m_bndry(:), 0);
    if numel(i_contour_all) == 0
        return;
    end

    contour_f_bndry = rltv_f_bndry(i_contour_all);
    contour_m_bndry = rltv_m_bndry(i_contour_all);

    v_rltv_rxn_in_bndry = inpolygon(rltv_f,rltv_m, contour_f_bndry, contour_m_bndry);
    attainable = all(v_rltv_rxn_in_bndry);

    if (attainable)
        % If the required relative reactions are fully within the relative
        % reactions of pressures along the 
        v_rltv_rxn_in_bndry = false(N_nodes, 1);
        v_rltv_rxn_in_bndry(1) = true;
        for j = 2 : N_nodes
            i_contour_j = boundary(rltv_f_bndry(j, :)', rltv_m_bndry(j, :)', 0.1);
            v_rltv_rxn_in_bndry(j) = inpolygon(rltv_f(j), rltv_m(j), rltv_f_bndry(j, i_contour_j), rltv_m_bndry(j, i_contour_j));
        end
    
        attainable = all(v_rxn_in_bndry) && all(v_rltv_rxn_in_bndry);
    end

    %% Functions
    function [traces_f, traces_m] = mat_wrenches_to_traces(mat_wrenches)
        N_ps = size(mat_wrenches, 3);
        N_nodes = size(mat_wrenches, 2);
    
        traces_f = zeros(N_nodes, N_ps);
        traces_m= zeros(N_nodes, N_ps);
        
        for i_node = 1 : N_nodes
            traces_f(i_node, :) = squeeze(mat_wrenches(1, i_node, :));
            traces_m(i_node, :) = squeeze(mat_wrenches(3, i_node, :));
        end
    end

end

