function [soln_mat_segment_twists, res] = solve_equilibrium_shape(N_segments, struct_design, p, Q_tip)
    default_segment_twists = zeros(3, N_segments);
    l_0 = 0.5;
    default_segment_twists(1, :) = l_0;
    default_segment_twists(3, :) = 1;

    opts = optimoptions("fsolve", MaxFunctionEvaluations=4e5, MaxIterations=5e3, StepTolerance=1e-8, FunctionTolerance=1e-8, Algorithm="levenberg-marquardt", Display="off");
    f = @(mat_in) check_equilibrium(mat_in, Q_tip, p, struct_design);
    [soln_mat_segment_twists, res] = fsolve(f, default_segment_twists, opts);
end
