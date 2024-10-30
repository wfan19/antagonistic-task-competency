function [p_soln, twists_soln, poses_soln, res, output] = find_p_minimize_pose_error(segment_twists, w_tip, struct_design, poses_to_count, args)

    arguments
        segment_twists
        w_tip
        struct_design
        poses_to_count = zeros(1, size(segment_twists, 2))
        args.p_0 = zeros(size(struct_design.p_bounds));
    end

    if all(poses_to_count == 0)
        poses_to_count(end) = 1;
    end

    N_actuators = length(struct_design.p_bounds);
    N_segments = size(segment_twists, 2);

    target_poses = calc_poses(struct_design.g_0, segment_twists);

    opts = optimoptions("fmincon");
    p_min = zeros(size(struct_design.p_bounds));
    [p_soln, res, exitflag, output] = fmincon(@cost, args.p_0, [], [], [], [], p_min, struct_design.p_bounds, [], opts);

    twists_soln= solve_equilibrium_shape(N_segments, struct_design, p_soln, w_tip);
    poses_soln = calc_poses(struct_design.g_0, twists_soln);
    
    function dist_cost = cost(p)
        eq_twists = solve_equilibrium_shape(N_segments, struct_design, p, w_tip);
        eq_poses = calc_poses(struct_design.g_0, eq_twists);

        %delta_tip_pose = Pose2.vee(inv(eq_poses(:, :, end)) * target_poses(:, :, end));

        K = diag([1, 1, 0.05]);
        %dist_cost = delta_tip_pose' * K * delta_tip_pose;

        errors = zeros(1, N_segments);
        for i_pose = find(poses_to_count)
            delta_pose = Pose2.vee(inv(eq_poses(:, :, i_pose)) * target_poses(:, :, i_pose));

            errors(i_pose) = delta_pose' * K * delta_pose;
        end

        dist_cost = dot(poses_to_count, errors);
    end

end

