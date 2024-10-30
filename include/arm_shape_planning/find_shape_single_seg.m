function [cell_g_circ_out, res] = find_shape_single_seg(pose_base, tip_poses, tip_load, N_nodes, f_cost, opts, mat_geo_0)
    arguments
        pose_base
        tip_poses
        tip_load
        N_nodes
        f_cost
        opts = optimoptions("fmincon", "MaxFunctionEvaluations", 3e4)
        mat_geo_0 = []
    end
    % NOTE: This function is heavily based on the
    % "find_base_curve_use_heuristics.m" function. However, we draw the
    % distinction between planning for a multi-segment VC arm versus a single 
    % VC segment. This is because the s-scaling has to be handled
    % differently - because we view each segment as an atomized unit, its s
    % should range from 0-1, even if it is only one part of the overall
    % arm. Thus, the length of the twist-vectors of any node should
    % correspond to the instantaneous length of its parent segment.

    % Given a set of target tip poses and loads, find the base curves that
    % optimize a given cost function
    
    % Note: here we employ a specific data structure for efficiently
    % encoding shear-free base curve geometries for arms with multiple
    % segments reaching multiple target poses. We are forced to do this
    % to input the data into Matlab's optimization functions. Normally, I
    % would prefer to work with a cell-array with each element being each
    % arm-series' matrix of twist vectors (mat_g_circ_right). Hence the
    % conversion function.

    % For a base-curve planning problem for an m-segment arm reaching
    % n-targets, the geometry matrix G is defined as 
    % [
    %   l_11, l_12, l_13, ..., l_1m;
    %   k_11, k_12, k_13, ..., k_1m;
    %   l_21, l_22, l_23, ..., l_2m;
    %   k_21, k_22, k_23, ..., k_2m;
    %   ...
    %   l_n1, l_n2, ...,       l_nm;
    %   k_n1, k_n2, ...,       k_nm;
    % ]
    %
    % where l_ij and k_ij are respectively the length and curvature of
    % segment i in the arm reaching target j.
    %
    % Additionally, each column in a geometry matrix (the lengths and
    % curvatures describing the segments of a single arm) is a "geometry
    % vector". 
    %

    %% Function body
    N_poses = size(tip_poses, 2);

    if isempty(mat_geo_0)
        v_geo_0 = repmat([0.1; 0], N_nodes, 1);
        mat_geo_0 = repmat(v_geo_0, 1, N_poses);
    end

    % Create the A and b matrix for applying a non-negative constraint on
    % the lengths
    % The A matrix just pulls out all the lengths from the vectorized
    % geometry matrix, such that
    % A * mat_geo(:) = [l_1, l_2, ..., l_n];
    N_states = N_nodes * N_poses * 2;
    A_select_ls = zeros(N_states / 2, N_states);
    for i_row = 1 : size(A_select_ls, 1)
        A_select_ls(i_row, 2*i_row - 1) = -1;
    end

    b_zero = zeros(N_states / 2, 1);

    [soln, res] = fmincon(f_cost, mat_geo_0, A_select_ls, b_zero, [], [], [], [], @base_curve_tip_constraint, opts);
    cell_g_circ_out = mat_geom_to_g_circ(soln);

    %% Cost functions
    % Constraint cost function
    function [ineq_residual, eq_residual] = base_curve_tip_constraint(mat_geom)
        % In:
        %   mat_geo: Geometry matrix describing lengths and curvatures of
        %            N_arms each with M segments
        % Out:
        %   ineq_residual: Inequality constraint residuals (= 0 )
        %   eq_residual: equality constraint residuals
        ineq_residual = 0;  % We don't have any inequality constraints

        N_poses = size(mat_geom, 2);
        N_nodes = size(mat_geom, 1);
        eq_residual = zeros(3, N_poses);
        cell_g_circ_right = mat_geom_to_g_circ(mat_geom);
        for i_arm = 1 : length(cell_g_circ_right)
            % Fetch the twist matrix of the current arm series
            mat_g_circ_right_i = cell_g_circ_right{i_arm};

            g_0 = Pose2.hat(pose_base);
            poses = calc_poses(g_0, mat_g_circ_right_i);
            tip_pose = poses(:, :, end);

            g_target_i = Pose2.hat(tip_poses(:, i_arm));
            rdelta_pose = inv(tip_pose) * g_target_i;
            K_weights = diag([1, 1, 0.1]);
            eq_residual(:, i_arm) = K_weights * Twist2.vee(logm(rdelta_pose));
        end
    end
    
    %% Helper functions
    function mat_g_circ_right = v_geom_to_g_circ(v_geom)
        % Convert a "geometry vector" to a matrix of twists
        % By "geometry vector" I mean [l_1; k_1; l_2; k_2; ...; l_n; k_n]
        n_g_circ = length(v_geom)/2;
        mat_l_k = reshape(v_geom, [2, n_g_circ]);
        lengths = mat_l_k(1, :);        % lengths
        shears = zeros(1, n_g_circ);    % shears
        curvatures = mat_l_k(2, :);     % curvatures
    
        mat_g_circ_right = [lengths; shears; curvatures];
    end

    function cell_g_circ_out = mat_geom_to_g_circ(mat_geom)
        % Convert a "geometry matrix" to a cell array of matrices of twists
        
        N_poses = size(mat_geom, 2);
        N_nodes = size(mat_geom, 1) / 2;
        cell_g_circ_out = cell(1, N_poses);
        for i = 1 : N_poses
            v_geom_i = mat_geom(:, i);
            cell_g_circ_out{i} = v_geom_to_g_circ(v_geom_i);
        end
    end


end

