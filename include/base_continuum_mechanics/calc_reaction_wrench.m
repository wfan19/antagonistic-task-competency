function [reaction_wrenches, forces] = calc_reaction_wrench(mat_segment_twists, p, struct_design)
    N_actuators = length(struct_design.fs);
    N_twists = size(mat_segment_twists, 2);
    lengths_o = mat_segment_twists(1, :);
    
    lengths = zeros(N_actuators, N_twists);
    strains = zeros(N_actuators, N_twists);
    forces = zeros(N_actuators, N_twists);

    curvatures = zeros(N_actuators, N_twists);
    moments = zeros(N_actuators, N_twists);
    for i = 1 : N_actuators
        lengths(i, :) = lengths_o - struct_design.rhos(i) * mat_segment_twists(3, :);
        strains(i, :) = (lengths(i, :) - struct_design.l_0) / struct_design.l_0;
        for j = 1 : length(strains)
            forces(i, j) = struct_design.fs{i}(strains(i, j), p(i));
        end

        curvatures(i, :) = mat_segment_twists(3, :) ./ lengths(i, :);
        moments(i, :) = (-1/3.5) * (p(i) / struct_design.p_bounds(i)) * curvatures(i, :);
        %moments(i, :) = (-0.25/3.5) * 1 * curvatures(i, :);
    end
    reaction_wrenches = struct_design.mat_A * forces;
    reaction_wrenches = reaction_wrenches + [0 0 0 0; 0 0 0 0; 1 1 1 1] * moments;
end