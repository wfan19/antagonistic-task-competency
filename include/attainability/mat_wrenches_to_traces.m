function [paths_f, paths_m] = mat_wrenches_to_traces(mat_wrenches)
    N_ps = size(mat_wrenches, 3);
    N_poses = size(mat_wrenches, 2);

    paths_f = zeros(N_poses, N_ps);
    paths_m= zeros(N_poses, N_ps);
    
    for i = 1 : N_poses
        paths_f(i, :) = squeeze(mat_wrenches(1, i, :));
        paths_m(i, :) = squeeze(mat_wrenches(3, i, :));
    end
end
