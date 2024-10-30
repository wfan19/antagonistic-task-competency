function plot_wrench_hull_loops(bndry_af_all, bndry_am_all, ax, linestyle)

    arguments
        bndry_af_all
        bndry_am_all
        ax
        linestyle = struct("color", [0.2941    0.4196    0.7490], "linewidth", 1)
    end
    
    overlapped = [];
    for i_node = 1 : size(bndry_af_all, 1)
        i_bndry_j = boundary(bndry_af_all(i_node, :)', bndry_am_all(i_node, :)', 0);
        if i_node == 1 
            overlapped = i_bndry_j;
        else
            overlapped = intersect(overlapped, i_bndry_j);
        end
        plot3(ax, bndry_af_all(i_node, i_bndry_j), bndry_am_all(i_node, i_bndry_j), i_node * ones(length(i_bndry_j)), linestyle);
    end

    % Plot the reaction sequences that are on the boundary for all of s.
    %plot3(ax, bndry_af_all(:, overlapped), bndry_am_all(:, overlapped), mat_z(:, overlapped), linestyle);
end

