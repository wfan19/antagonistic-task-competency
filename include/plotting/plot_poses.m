function plot_poses(poses, ax, linestyle, plot_pose_markers, args)
    arguments
        poses
        ax
        linestyle="k-";
        plot_pose_markers=true
        args.axis_length = 0.15;
        args.pose_markers_colored=true;
    end

    color_green = [45 182 125] / 255;
    color_yellow = [255 176 0] / 255;
    color_blue = [100 143 255] / 255;
    color_red = [255 48 150] / 255;
    color_gray = [150, 150, 150]/255;
    
    % Extract position components
    p_x = squeeze(poses(1, 3, :));
    p_y = squeeze(poses(2, 3, :));

    % Plot
    hold(ax, "on")
    plot(ax, p_x, p_y, linestyle);

    if plot_pose_markers
        % Extract x unit vector components
        x_u = squeeze(poses(1, 1, :));
        x_v = squeeze(poses(2, 1, :));
    
        y_u = squeeze(poses(1, 2, :));
        y_v = squeeze(poses(2, 2, :));

        if args.pose_markers_colored
            quiver(ax, p_x, p_y, x_u, x_v, args.axis_length, "color", color_red, "linewidth", 2, "HandleVisibility","off");
            quiver(ax, p_x, p_y, y_u, y_v, args.axis_length, "color", color_green, "linewidth", 2, "HandleVisibility", "off");
        else
            quiver(ax, p_x, p_y, x_u, x_v, args.axis_length, "color", color_gray, "linewidth", 2, "HandleVisibility","off");
            quiver(ax, p_x, p_y, y_u, y_v, args.axis_length, "color", color_gray, "linewidth", 2, "HandleVisibility", "off");
        end
    end
    axis(ax, "equal")
end