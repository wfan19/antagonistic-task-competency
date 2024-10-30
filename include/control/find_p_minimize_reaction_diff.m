function [min_residual, best_p] = find_p_minimize_reaction_diff(segment_twists, w_tip, struct_design)
    N_actuators = length(struct_design.p_bounds);

    f_reaction_dist = @(pres) check_equilibrium_norm_modified(segment_twists, w_tip, pres, struct_design);
    opts = optimoptions('fmincon', 'display', 'off', 'StepTolerance', 1e-20, 'FunctionTolerance', 1e-10); 

    % Set up multiple-start initial points
    N_starts = 5;
    v_zero = zeros(N_actuators, 1);
    start_points = diag(struct_design.p_bounds) * rand([N_actuators, N_starts - 2]);
    start_points = [start_points, v_zero, struct_design.p_bounds];
    
    % Solve the optimization problem
    p_soln_starts = zeros(N_actuators, N_starts);
    min_residuals_i = zeros(N_starts, 1);
    for i_start = 1 : N_starts
        p_start = start_points(:, i_start);
        [p_soln_starts(:, i_start), min_residuals_i(i_start), ~, output] = fmincon(f_reaction_dist, p_start, [], [], [], [], v_zero, struct_design.p_bounds, [], opts); 
    end
    
    % Of the result from each starting point, find the lowest one and return them.
    [min_residual, i_start_min] = min(min_residuals_i);
    best_p = p_soln_starts(:, i_start_min);

    %% Cost function for the residuals. Weights the rotatonal residuals higher than the linear ones
    function residuals_out = check_equilibrium_norm_modified(segment_twists, w_tip, pressure, struct_design)
        residuals = check_equilibrium(segment_twists, w_tip, pressure, struct_design);
    
        weights_stretch_vs_bending = [1, 1, 10];
        weighted_residuals = diag(weights_stretch_vs_bending) * residuals;
    
        residuals_out = sum(vecnorm(weighted_residuals, 1));
    end

end