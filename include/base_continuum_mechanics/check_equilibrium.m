function residual = check_equilibrium(mat_segment_twists, Q_tip, p, struct_design)
    % Compute the externally applied wrenches
    external_wrenches = calc_external_wrench(mat_segment_twists, Q_tip, struct_design.g_0);

    % Compute the internal reaction wrenches
    reaction_wrenches = calc_reaction_wrench(mat_segment_twists, p, struct_design);

    residual = external_wrenches + reaction_wrenches;

    % Apply a shear-removing term
    % TODO: account for this in the Jacobian!!
    % Need to set thigns like J_e(2,2), J_e(2, 5), ..., J_e(3*i+2, 3*j+2) 
    % to 1e5
    residual(2, :) = 1e3 * mat_segment_twists(2, :);
end