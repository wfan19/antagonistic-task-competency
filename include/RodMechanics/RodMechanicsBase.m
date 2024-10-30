classdef RodMechanicsBase
    % Base class of any rod mechanics model
    % A rod mechanics model encompasses how an elastic rod behaves on its
    % own. Thus it may include any of the following:
    % - Neutral length
    % - Force produced as a function of strain/pressure (force-surface).
    % - Linear stiffness
    % - Bending stiffness
    % - Stiffness matrix
    
    properties
        l_0 = 1
        strain = 0
        f_force = @f_force_default
        
        e_bounds = [-0.5, 0.5]   % Bounds on the strain
        a_bounds = [0, 50]   % Bounds on the actuation/pressure values
    end
    
  methods
        function obj = RodMechanicsBase(l_0, f_force)
            arguments
                l_0 = 1;
                f_force = @f_force_default;
            end

            obj.l_0 = l_0;
            obj.f_force = f_force;
        end
        
        % We assume by default that an actuator's force is entirely
        % characterized by the strain and input.
        function force = f_force_default(obj, strain, actuation)
            error("Attempting to call force function of a base class rod function!")
        end

        % Compute the force that the actuator would generate if actuated,
        % given the current strain.
        function force = get_force(obj, actuation)
            force = obj.f_force(obj.strain, actuation);
        end
        
        % Update the actuator's strain based on a new length
        % TODO: This pattern feels a little weird - is this the best way to
        % do this?
        function obj = update_strain(obj, l)
            obj.strain = (l - obj.l_0) / obj.l_0;
        end

        % Plot the actuator's force surface
        function plot_force_surface(obj, e_bounds, a_bounds, ax, options)
            arguments
                obj
                e_bounds = obj.e_bounds
                a_bounds = obj.a_bounds
                ax = axes(figure())
                options.resolution = 40
            end

            strains = linspace(e_bounds(1), e_bounds(2), options.resolution);
            actuations = linspace(a_bounds(1), a_bounds(2), options.resolution);
            
            [E, A] = meshgrid(strains, actuations);
            F = zeros(options.resolution, options.resolution);
            for i = 1 : options.resolution
                for j = 1 : options.resolution
                    strain_i = E(i,j);
                    actuation_i = A(i, j);
                    F(i, j) = obj.f_force(strain_i, actuation_i);
                end
            end
            
            % TODO:
            % 1) Recolor the different actuation zones
            % 2) Draw the different meaningful lines:
            %     - Passive extension/contraction
            %     - Free movement
            %     - Blocked force
            mesh(ax, E, A, F);
            xlabel("Strain (%)")
            ylabel("Pressure (kpa)")
            zlabel("Force (N)")

            free_contraction = contourc(strains, actuations, F, [0,0]);
            free_contraction = free_contraction(:, 2:end);
            hold on
            plot3(free_contraction(1, :), free_contraction(2, :), zeros(length(free_contraction)), "k", "linewidth", 3);
        end
        
    end
end

