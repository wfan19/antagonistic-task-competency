function dist_cost = cost_pose_error(p, struct_design, target_poses, poses_to_count)
    N_segments = length(target_poses) - 1;
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