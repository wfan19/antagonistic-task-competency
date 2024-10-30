classdef BasicBellowMechanics < RodMechanicsBase
    
    methods
        function obj = BasicBellowMechanics(l_0)
            arguments
                l_0 = 1
            end
            f_force = BasicBellowMechanics.make_bellow_force_func();
            obj@RodMechanicsBase(l_0, f_force);

            obj.e_bounds = [-0.142, 0.142];
            obj.a_bounds = [0, 70];
        end
    end

    methods(Static)
        function [fitresult, X] = make_bellow_force_func()
            % TODO: Make this parameterized by keypoints so we can do
            % inverse design!

            % Sign convention: positive force means pushing outward.

            % Define keypoints
            x0 = [0; 0; 0];
            x1 = [-0.142; 0; 1.1];    % Passive compression
            x2 = [0; 70; 100];         % Force when inflated and length unchanged
            x3 = [0.50; 20; 0];       % Free extension line
            x4 = [0.142; 0; -3];      % Passive extension
        
            % Calculate slopes of each line
            k1 = x1(3) / x1(1);
            k2 = x2(3) / x2(2);
            k3 = x3(2) / x3(1);
            k4 = x4(3) / x4(1);
        
            % Define interpolation bounds [min, max]
            e_bounds = [-0.3, 0.5];
            p_bounds = [0, 70];
            
            % Add interpolation corners
            f_f5 = k1 * e_bounds(1) + k2 * p_bounds(2);
            f_f6 = k2 * p_bounds(2) - k2 * k3 * e_bounds(2);
        
            % Rebuild the points now around the bounds
            x1 = [e_bounds(1); 0; k1 * e_bounds(1)];
            x2 = [0; p_bounds(2); k2 * p_bounds(2)];
            x3 = [e_bounds(2); k3 * e_bounds(2); 0];
            x4 = [e_bounds(2); 0; k4 * e_bounds(2)];
            x5 = [x1(1); x2(2); f_f5];
            x6 = [x3(1); x2(2); f_f6];
            
            X = [x0, x1, x2, x3, x4, x5, x6];
            
            f_e = X(1, :);
            f_p = X(2, :);
            f_f = X(3, :);
            
            [xData, yData, zData] = prepareSurfaceData( f_e, f_p, f_f );
            
            % Set up fittype and options.
            ft = 'cubicinterp';
            
            % Fit model to data.
            [fitresult, gof] = fit( [xData, yData], zData, ft, 'Normalize', 'on' );
        end
    end
end

