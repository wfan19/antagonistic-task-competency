function in_bndry = check_loads_attainable(bndry_af, bndry_am, test_f, test_m)

    N_tests = size(test_f, 2);
    N_nodes = size(bndry_af, 1);

    in_bndry = false(size(test_f));

    for i = 1 : N_nodes
        % Obtain the half-space representation of the attainable wrench
        % hull at each point s along the arm
        [A_i, b_i] = vert2lcon([bndry_af(i, :)', bndry_am(i, :)']);

        % Format the test force and moments into a set of test points in R^2
        test_pts_i = [test_f(i, :); test_m(i, :)];

        % Determine if they lie in the convex hull by using the half-space
        % representation.
        in_bndry(i, :) = all(A_i * test_pts_i < b_i, 1);
    end

end

