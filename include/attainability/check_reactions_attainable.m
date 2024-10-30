function [v_attainable, min_dists, p_solns] = check_reactions_attainable(q_tests, segment_twists, struct_design, p_bounds, threshold)
    % Check if a series of reactions are actually attainable by an arm
    % Inputs:
    %   - q_tests (3 x N_segments x N_test_points): a set of test reaction
    %   requirements at each point along the arm
    %   - segment_twists (3 x N_segments): twist vector at each segment
    %   - struct_design (struct): standard arm design struct
    %   - p_bounds (N_actuators x 1): max pressures for each actuator
    %
    % Outputs:
    %   - v_attainable (N_test_points x 1): boolean vector indicating
    %   whether each test arm reaction requirements are attainable
    N_tests = size(q_tests, 3);
    N_actuators = length(p_bounds);

    v_attainable = false(N_tests, 1);
    min_dists = zeros(N_tests, 1);
    p_solns = zeros(length(p_bounds), N_tests);

    N_starts = 5;
    v_zero = zeros(N_actuators, 1);
    start_points = diag(p_bounds) * rand([N_actuators, N_starts - 2]);
    start_points = [start_points, v_zero, p_bounds];

    for i = 1 : N_tests
        % Define the cost function: for each pressure, find how close it
        % gets the reactions to those required.
        f_reaction_dist = @(pres) check_equilibrium_norm_modified(pres, q_tests(:, :, i), segment_twists);
        opts = optimoptions('fmincon', 'display', 'off', 'StepTolerance', 1e-20, 'FunctionTolerance', 1e-10); 

        % Solve the optimization problem
        p_soln_starts = zeros(N_actuators, N_starts);
        min_dists_starts = zeros(N_starts, 1);
        for i_start = 1 : N_starts
            p_start = start_points(:, i_start);
            [p_soln_starts(:, i_start), min_dist_starts(i_start), ~, output] = fmincon(f_reaction_dist, p_start, [], [], [], [], v_zero, p_bounds, [], opts); 
        end

        [min_dists(i), i_start_min] = min(min_dist_starts);
        p_solns(:, i) = p_soln_starts(:, i_start_min);

        % If the minimum distance is below a threshold, then the reaction
        % is reachable and thus return true.
        v_attainable(i) = min_dists(i) < threshold;
    end
end

