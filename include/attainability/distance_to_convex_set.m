function [min_dist, closest_pt, weights] = distance_to_convex_set(point, vertices, K)

    arguments
        point
        vertices
        K = ones(size(point));
    end

    % Find the point in a convex set that minimizes the distance to a given
    % point, and return both the distance and the point's coordinates.
    %
    % We do this by solving a quadratic program with convex linear costs
    %
    % We want to solve:
    %   min_y ||x - y||             (where ||.|| is the euclidean norm)
    %   s.t. y = vertices * weights (ensure y is a weighted sum of vertices)
    %        sum(weights) <= 1      (ensure y is a convex combination of vertices)
    %
    % To solve this with fmincon, we simply substitute the first constraint
    % into the cost function and use the weights as the decision variables
    % to yield the following problem:
    %
    %   min_w ||x - vertices * weights||
    %   s.t. sum(weights) = 1
    %
    % This is the form of the problem implemented below

    assert(size(vertices, 1) == 2, "Ensure vertices is a 2D column matrix");
    
    cost_func = @(v_weights) (point - (vertices * v_weights))' * diag(K) * (point - (vertices * v_weights));
    
    % Construct the convex-combination constraint
    constraint_A = ones(1, size(vertices, 2));
    constraint_b = 1;

    lb = zeros(size(vertices, 2), 1);

    % Solve the problem with fmincon
    weights_0 = zeros(size(vertices, 2), 1);
    opts = optimoptions("fmincon", "display", "notify");
    [weights, min_dist] = fmincon(cost_func, weights_0, [], [], constraint_A, constraint_b, lb, [], [], opts); 

    closest_pt = vertices * weights;
end

