function create_tube_triangle_mesh(pts_loop_1, pts_loop_2, ax, face_color, face_alpha)
    arguments
        pts_loop_1
        pts_loop_2
        ax
        face_color = [100 143 255] / 255
        face_alpha = 0.6 
    end

    
    N_pts_1 = size(pts_loop_1, 2);
    N_pts_2 = size(pts_loop_2, 2);
    
    % Create vertex arrays
    pts_all = [pts_loop_1, pts_loop_2];

    % Initialize faces array
    faces = [];

    % Connect points using triangle faces
    for i = 1:max(N_pts_1, N_pts_2)
        % Determine the indices for the two circles (wrapping around if needed)
        i1 = mod(i-1, N_pts_1) + 1;
        i2 = mod(i, N_pts_1) + 1;
        j1 = mod(i-1, N_pts_2) + 1;
        j2 = mod(i, N_pts_2) + 1;
        
        % Define triangles that connect the points between circles
        if N_pts_1 <= N_pts_2
            % If circle 1 has fewer points, loop through it and add triangles
            faces = [faces; i1, N_pts_1+j1, N_pts_1+j2; i1, N_pts_1+j2, i2];
        else
            % If circle 2 has fewer points, loop through it and add triangles
            faces = [faces; N_pts_2+i1, j1, j2; N_pts_2+i1, j2, N_pts_2+i2];
        end
    end

    % Plot the triangular mesh
    trisurf(faces, pts_all(1, :), pts_all(2, :), pts_all(3, :), "parent", ax, "facecolor", face_color, "facealpha", face_alpha, "edgealpha", 0);
end