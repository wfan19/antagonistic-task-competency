function tests = test_mechanics
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

function helper_test_antagonist_arm(p, q_tip, testCase)
    %%% Test scenario
    % Make the arm design struct
    r_outer = 0.08;
    r_inner = 0.015;
    struct_design = make_antagonist_arm(r_outer, r_inner);

    N_nodes = 10;
    eq_twists = solve_equilibrium_shape(N_nodes, struct_design, p, q_tip);
    eq_poses = calc_poses(struct_design.g_0, eq_twists);
    
    plot_poses(eq_poses, axes(figure()));

    tip_pose = Pose2.vee(eq_poses(:, :, end));
    color_yellow = [255 176 0] / 255;
    uv = q_tip(1:2) * 0.01;
    quiver(tip_pose(1), tip_pose(2), uv(1), uv(2), "color", color_yellow);
end

function ax = sweep_antagonism(q_tip, p_hold, i_change, N_ps, testCase)
    %%% Test scenario
    % Make the arm design struct
    r_outer = 0.08;
    r_inner = 0.015;
    struct_design = make_antagonist_arm(r_outer, r_inner);

    % Pressure and loading
    N_nodes = 10;

    ps_to_test = linspace(0, 50, N_ps);

    % Make a plot sweeping the pressure of the left bellow
    cell_twists = cell(size(ps_to_test));
    residuals = zeros(size(ps_to_test));
    cell_poses = cell(size(ps_to_test));
    ax = axes(figure());
    for i_p = 1 : length(ps_to_test)
        p = p_hold;
        p(i_change) = ps_to_test(i_p);

        [cell_twists{i_p}, mat_res_i] = solve_equilibrium_shape(N_nodes, struct_design, p, q_tip);
        cell_poses{i_p} = calc_poses(struct_design.g_0, cell_twists{i_p});

        residuals(i_p) = norm(mat_res_i);
        if residuals(i_p) > 1e-5
            plot_poses(cell_poses{i_p}, ax, struct("color", [0.5 0.5 0.5]), "pose_markers_colored", false);
        else
            plot_poses(cell_poses{i_p}, ax);
        end
        
        tip_pose = Pose2.vee(cell_poses{i_p}(:, :, end));
        color_yellow = [255 176 0] / 255;
        uv = q_tip(1:2) * 0.01;
        quiver(tip_pose(1), tip_pose(2), uv(1), uv(2), "color", color_yellow);
    end
end

function test_antagonism1(testCase)
    % Pressure and loading
    q_tip = [3.1525; 5.5658; 0.5592];
    p = [0.99; 0.99; 0.99; 0.99];    
    
    helper_test_antagonist_arm(p, q_tip, testCase)
end

function test_antagonism1_sweep(testCase)
    q_tip = [3.1525; 5.5658; 0.5592];
    p_hold = [0.99; 0.99; 0.99; 0.99];    
    ax = sweep_antagonism(q_tip, p_hold, 1, 20, testCase);
    title(ax, "Sweep left bellow pressure 0-50kpa")

    ax = sweep_antagonism(q_tip, p_hold, 4, 20, testCase);
    title(ax, "Sweep right bellow pressure 0-50kpa")
end


function test_antagonism2(testCase)
    %%% Test scenario
    % Make the arm design struct
    r_outer = 0.08;
    r_inner = 0.015;
    struct_design = make_antagonist_arm(r_outer, r_inner);

    % Pressure and loading
    N_nodes = 10;
    q_tip = [-9.972; -2.106; -0.0157];

    N_ps = 10;
    ps_to_test = linspace(0, 50, N_ps);

    % Make a plot sweeping the pressure of the left bellow
    cell_twists = cell(size(ps_to_test));
    cell_poses = cell(size(ps_to_test));
    ax = axes(figure());
    title("Sweep left bellow pressure 0-50kpa")
    for i_p = 1 : length(ps_to_test)
        p = [ps_to_test(i_p); 0.99; 0.99; 0.99];

        cell_twists{i_p} = solve_equilibrium_shape(N_nodes, struct_design, p, q_tip);
        cell_poses{i_p} = calc_poses(struct_design.g_0, cell_twists{i_p});

        plot_poses(cell_poses{i_p}, ax)
        
        tip_pose = Pose2.vee(cell_poses{i_p}(:, :, end));
        color_yellow = [255 176 0] / 255;
        uv = q_tip(1:2) * 0.01;
        quiver(tip_pose(1), tip_pose(2), uv(1), uv(2), "color", color_yellow);
    end

    % Make a plot sweeping the pressure of the right bellow
    cell_twists = cell(size(ps_to_test));
    cell_poses = cell(size(ps_to_test));
    ax = axes(figure());
    title("Sweep right bellow pressure 0-50kpa")
    for i_p = 1 : length(ps_to_test)
        p = [0.99; 0.99; 0.99; ps_to_test(i_p)];

        cell_twists{i_p} = solve_equilibrium_shape(N_nodes, struct_design, p, q_tip);
        cell_poses{i_p} = calc_poses(struct_design.g_0, cell_twists{i_p});

        plot_poses(cell_poses{i_p}, ax)
        
        tip_pose = Pose2.vee(cell_poses{i_p}(:, :, end));
        color_yellow = [255 176 0] / 255;
        uv = q_tip(1:2) * 0.01;
        quiver(tip_pose(1), tip_pose(2), uv(1), uv(2), "color", color_yellow);
    end
end