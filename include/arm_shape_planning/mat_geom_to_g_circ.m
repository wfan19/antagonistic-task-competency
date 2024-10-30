function g_circ_out = mat_geom_to_g_circ(mat_geom)
    % Convert a "geometry matrix" to a cell array of matrices of twists
    
    N_poses = size(mat_geom, 2);
    N_segs = size(mat_geom, 1) / 2;
    cell_g_circ_out = cell(1, N_poses);
    for i = 1 : N_poses
        v_geom_i = mat_geom(:, i);
        cell_g_circ_out{i} = v_geom_to_g_circ(v_geom_i);
    end
    g_circ_out = [cell_g_circ_out{:}];
end