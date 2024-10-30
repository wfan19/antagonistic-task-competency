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