function plot_wrench_hull_trimesh(bndry_af_all, bndry_am_all, zs, ax, face_color, face_alpha)
    arguments
        bndry_af_all
        bndry_am_all
        zs
        ax
        face_color = [100 143 255] / 255
        face_alpha = 0.6 
    end

    face_vert_indices = [];
    
    N_nodes = size(bndry_af_all, 1);
    hold(ax, "on")
    for i_node = 1 : N_nodes - 1
        % Find the convex hull of the i-th attainable wrench space
        i_bndry = boundary(bndry_af_all(i_node, :)', bndry_am_all(i_node, :)', 0);
        i_bndry_next = boundary(bndry_af_all(i_node+1, :)', bndry_am_all(i_node+1, :)', 0);
        
        pts_wrench_hull_i = [bndry_af_all(i_node, i_bndry); bndry_am_all(i_node, i_bndry); zs(i_node, i_bndry)];
        pts_wrench_hull_next = [bndry_af_all(i_node+1, i_bndry_next); bndry_am_all(i_node+1, i_bndry_next); zs(i_node+1, i_bndry_next)];
        create_tube_triangle_mesh(pts_wrench_hull_i, pts_wrench_hull_next, ax, face_color, face_alpha);
    end
end

