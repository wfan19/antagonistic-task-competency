function [external_wrenches, J] = calc_external_wrench(mat_segment_twists, Q_tip, g_0)
    N_twists = size(mat_segment_twists, 2);
    poses = calc_poses(g_0, mat_segment_twists);
    g_tip = poses(:, :, end);
    
    external_wrenches = zeros(3, N_twists, class(mat_segment_twists));
    for i = 1 : N_twists
        g_i = poses(:, :, i);
        g_i_tip = inv(g_i) * g_tip;
        
        % Convert the world frame tip force to be in the rod tip's frame
        g_ucirc_right_tip = Pose2.left_lifted_action(g_tip)' * Q_tip;

        % Since the force is now in the rod's "material-centric" frame,
        % we can therefore use the adjoint to find its contribution on an
        % earlier part of the rod.
        g_ucirc_right_s = inv(Pose2.adjoint(g_i_tip))' * g_ucirc_right_tip;
    
        external_wrenches(:, i) = g_ucirc_right_s;
    end
end