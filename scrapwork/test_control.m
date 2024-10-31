function tests = test_control
    tests = functiontests(localfunctions);
end

function struct_antagonist = make_antagonist_arm(r_outer, r_inner)
    %%% Define arm design
    % Actuator position radii and the associated moment-arm matrix
    rs = [-r_outer, -r_inner, r_inner, r_outer];
    mat_A = [
        1, 1, 1, 1;
        0, 0, 0, 0;
        -rs;
        ];
    
    l_0 = 0.5;
    
    g_0 = Pose2.hat([0, 0, -pi/2]);
    
    % Assemble it all into structs
    struct_design_base = struct();
    struct_design_base.mat_A = mat_A;
    struct_design_base.rhos = rs;
    struct_design_base.l_0 = l_0;
    struct_design_base.g_0 = g_0;
    
    % Muscle force functions
    f_bellow = @FittedBellowMechanics.actuator_force;
    f_muscle = @FittedMuscleMechanics.actuatorForce_key;
    
    struct_antagonist = struct_design_base;
    struct_antagonist.fs = {f_bellow, f_muscle, f_muscle, f_bellow};
    struct_antagonist.p_bounds = [50; 100; 100; 50];
end

function struct_4bellow = make_bellows_arm(r_outer, r_inner)
    %%% Define baseline parameters:
    
    % Actuator position radii and the associated moment-arm matrix
    rhos = [-r_outer, -r_inner, r_inner, r_outer];
    mat_A = [
        1, 1, 1, 1;
        0, 0, 0, 0;
        -rhos;
        ];
    
    l_0 = 0.5;
    
    g_0 = Pose2.hat([0, 0, -pi/2]);
    N_nodes = 10;
    
    % Assemble it all into structs
    struct_design_base = struct();
    struct_design_base.mat_A = mat_A;
    struct_design_base.rhos = rhos;
    struct_design_base.l_0 = l_0;
    struct_design_base.g_0 = g_0;
    
    % Muscle force functions
    f_bellow = @FittedBellowMechanics.actuator_force;
    f_muscle = @FittedMuscleMechanics.actuatorForce_key;
    
    %%% Create variations 
    struct_4bellow= struct_design_base;
    struct_4bellow.fs = {f_bellow, f_bellow, f_bellow, f_bellow};
    struct_4bellow.p_bounds = [50; 50; 50; 50];
end

function helper_test_min_pose_error(q_tip, struct_design, testCase)
    tic

    % Target shape
    N_nodes = 10;
    s = linspace(0, 1, N_nodes);
    a = -3;
    b = 3;
    l_0 = 0.5;
    segment_twists_target = [
        l_0*ones(size(s)); % TODO: This seems suspect - but for these tests it's really hard to consider what this should be
        zeros(size(s));
        a*s + b
    ];
    poses_target = calc_poses(struct_design.g_0, segment_twists_target);

    % Pressure and loading
    N_nodes = 10;

    tic
    p_0 = [10; 0; 10; 0];
    [p_soln, twists_soln, poses_soln, res, out] = find_p_minimize_pose_error(segment_twists_target, q_tip, struct_design, p_0=p_0);
    toc

    % Check equilibrium poses resulting from pressures sampled from the
    % edges of the pressure space.
    p_sampled= sample_edges_of_cuboid(3, struct_design.p_bounds);
    % N_samples = 200;
    % p_sampled = diag(struct_design.p_bounds) * betarnd(0.3, 0.3, 4, N_samples);
    poses_sampled = cell(1, size(p_sampled, 2));
    cell_residuals = cell(1, size(p_sampled, 2));
    for i = 1 : size(p_sampled, 2)
        p_i = p_sampled(:, i);
        [segment_twists_i, cell_residuals{i}] = solve_equilibrium_shape(N_nodes, struct_design, p_i, q_tip);
        poses_sampled{i} = calc_poses(struct_design.g_0, segment_twists_i);
    end

    struct_target_line = struct("color", [0 0 0]);
    struct_sample_line = struct("color", [150, 150, 150] / 255);

    ax = axes(figure());
    plot_poses(poses_target, ax, struct_target_line);

    plot_poses(poses_soln, ax, struct("color", "b"));
    tip_pose = Pose2.vee(poses_soln(:, :, end));
    color_yellow = [255 176 0] / 255;
    uv = q_tip(1:2) * 0.01;
    quiver(tip_pose(1), tip_pose(2), uv(1), uv(2), "color", color_yellow);

    for i = 1 : length(poses_sampled)
        plot_poses(poses_sampled{i}, ax, struct_sample_line, false);
        
        tip_pose = Pose2.vee(poses_sampled{i}(:, :, end));
        color_yellow = [255 176 0] / 255;
        uv = q_tip(1:2) * 0.01;
        quiver(tip_pose(1), tip_pose(2), uv(1), uv(2), "color", color_yellow);
    end
    
    norm_residuals = cellfun(@(R) norm(R), cell_residuals);
    for i = find(norm_residuals > 1e-5)
        plot_poses(poses_sampled{i}, ax, struct("color", "r"), false);
    end
    disp(p_soln)
    toc
end

function test_min_pose_error_case_1(testCase)
    % The case where the closest attainable arm shape to the target shape
    % is for some reason pointing way off to the left
    q_tip = [3.1525; 5.5658; 0.5592];

    r_outer = 0.08;
    r_inner = 0.015;
    struct_design = make_antagonist_arm(r_outer, r_inner);

    helper_test_min_pose_error(q_tip, struct_design, testCase);
end

function test_min_pose_error_case_2(testCase)
    % The case where the closest attainable arm shape to the target shape
    % is for some reason pointing all the way up.
    q_tip = [-9.972; -2.106; -0.0157];

    r_outer = 0.08;
    r_inner = 0.015;
    struct_design = make_antagonist_arm(r_outer, r_inner);

    helper_test_min_pose_error(q_tip, struct_design, testCase);
end

function test_min_pose_error_case_3(testCase)
    q_tip = [-8.5295; -8.7823; -0.7772];

    r_outer = 0.08;
    r_inner = 0.015;
    struct_design = make_bellows_arm(r_outer, r_inner);

    helper_test_min_pose_error(q_tip, struct_design, testCase);
end

function test_min_pose_error_case_4(testCase)
    q_tip = [9.2328; 6.7523; 0.0403];

    r_outer = 0.08;
    r_inner = 0.015;
    struct_design = make_bellows_arm(r_outer, r_inner);

    helper_test_min_pose_error(q_tip, struct_design, testCase);
end

