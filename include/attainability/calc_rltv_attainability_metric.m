function [min_dists, closest_pts, weights] = calc_rltv_attainability_metric(w_f, w_m, bndry_rltv_af, bndry_rltv_am)
    % For each given reaction requirement (qf, qm), compute the closest
    % distance between it and each of the convex hulls that it must lie
    % within. 
    %
    % Inputs:
    %   - qf: reaction requirements' force components. Columns are
    %       individual requirements, rows indicate position along arm
    %   - qm: same but for the moment component
    %
    %   - bndry_rltv_af: force component of the relative traces that bound
    %   the relative attainable reaction space
    %   - bndry_rltv_am: sasme but for the moments
    
    % Get the dimensionality
    N_poses = size(w_f, 1);
    N_test_qs = size(w_f, 2);

    % Preallocate outputs
    min_dists = zeros(N_poses, N_test_qs);
    closest_pts = zeros(2, N_poses, N_test_qs);
    weights = cell(N_poses, N_test_qs);

    % For each requirement
    for i_q = 1 : N_test_qs
        q_i = [w_f(:, i_q)'; w_m(:, i_q)'];
        for i = 2 : N_poses
            % Get the convex hull of the vertices
            % NOTE: computing the closest distance/points does not require
            % its input be the convex hull, and it still works if the list
            % of vertices contains interior points or repeats. However,
            % getting the convex hull can significantly reduce the
            % dimensionality of the problem and thus drastically speed it
            % up. We noticed the time took 40% the time it originally took.
            vertices_i = [bndry_rltv_af(i, :); bndry_rltv_am(i, :)];
            i_conv_hull = boundary(vertices_i(1, :)', vertices_i(2, :)', 0);
            conv_hull_vertices_i = vertices_i(:, i_conv_hull);
            
            [min_dists(i, i_q), closest_pts(:, i, i_q), weights{i, i_q}] = distance_to_convex_set(q_i(:, i), conv_hull_vertices_i);
        end
    end
end

