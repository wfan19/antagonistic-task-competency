function ax = plot_wrench_hull(bndry_af, bndry_am, ax, color, plot_style, mat_z)

    arguments
        bndry_af
        bndry_am
        ax = axes(figure());
        color = [255 48 150] / 255;
        plot_style = struct("facealpha", 0.6, "linewidth", 1)
        mat_z = repmat(transpose(1:size(bndry_af, 1)), 1, size(bndry_af, 2));
    end
    N_poses = size(bndry_af, 1);
    N_rxns = size(bndry_af, 2);
    
    X = zeros(4, N_rxns * (N_poses - 1));
    Y = zeros(4, N_rxns * (N_poses - 1));
    Z = zeros(4, N_rxns * (N_poses - 1));
    N_rxns = size(bndry_af, 2);
    for i = 1 : N_poses - 1
        X_i = [
            bndry_af(i, :);
            circshift(bndry_af(i, :), 1);
            circshift(bndry_af(i+1, :), 1);
            bndry_af(i+1, :);
        ];
    
        Y_i = [
            bndry_am(i, :);
            circshift(bndry_am(i, :), 1);
            circshift(bndry_am(i+1, :), 1);
            bndry_am(i+1, :);
        ];
    
        Z_i = [
            mat_z(i, :);
            mat_z(i, :);
            mat_z(i+1, :);
            mat_z(i+1, :);
        ];
    
        X(:, N_rxns * (i-1) + 1 : N_rxns*i) = X_i;
        Y(:, N_rxns * (i-1) + 1 : N_rxns*i) = Y_i;
        Z(:, N_rxns * (i-1) + 1 : N_rxns*i) = Z_i;
    end

    fill3(ax, X, Y, Z, color, plot_style, "handlevisibility", "off");
end

