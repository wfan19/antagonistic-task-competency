function poses = calc_poses(g_0, mat_segment_twists)
    N_twists = size(mat_segment_twists, 2);
    ds = 1/N_twists;
    N_poses = N_twists + 1;

    poses = zeros(3, 3, N_poses, class(mat_segment_twists));
    poses(:, :, 1) = g_0;
    for i = 1 : N_twists

        % Get the current twist
        segment_twist_s = mat_segment_twists(:, i);

        % Compute the next displacement
        delta_g = Twist2.expm(segment_twist_s*ds);

        % Apply the next displacement
        poses(:, :, i+1) = poses(:, :, i) * delta_g;
    end
end