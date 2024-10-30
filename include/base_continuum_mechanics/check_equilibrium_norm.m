function dist = check_equilibrium_norm(segment_twists, q_test, pressure, struct_design)
        reactions = calc_reaction_wrench(segment_twists, pressure, struct_design);
        residuals = q_test + reactions;
        
        residuals(2, :) = 1e3 * segment_twists(2, :);

        %dist = sum(vecnorm(residuals, 1));
        dist = norm(residuals(:));
    
end