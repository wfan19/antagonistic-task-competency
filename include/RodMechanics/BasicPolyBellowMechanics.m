classdef BasicPolyBellowMechanics < RodMechanicsBase
    
    methods
        function obj = BasicPolyBellowMechanics(l_0)
            arguments
                l_0 = 1
            end
            f_force = @BasicPolyBellowMechanics.actuator_force;
            obj@RodMechanicsBase(l_0, f_force);
            
            obj.e_bounds = [-0.142, 0.142];
            obj.a_bounds = [0, 70];
        end
    end

    methods(Static)
        function fitresult = make_poly_bellow_force_func()
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
            
            % Add in more points along the lines we know we need to fit to
            x7 = [0; p_bounds(2)/2; k2 * p_bounds(2)/2];
            x8 = [e_bounds(2)/2; k3 * e_bounds(2)/2; 0];
            x9 = [e_bounds(2) / 2; 0; k4 * e_bounds(2) / 2];
            
            
            X = [x0, x1, x2, x3, x4, x5, x6, x7, x8, x9];
            
            f_e = X(1, :);
            f_p = X(2, :);
            f_f = X(3, :);
            
            [xData, yData, zData] = prepareSurfaceData( f_e, f_p, f_f );
            
            % Set up fittype and options.
            ft = fittype( 'poly33' );
            
            % Fit model to data.
            [fitresult, ~] = fit( [xData, yData], zData, ft );
        end

        function force = actuator_force(strain, pressure)
            % Hardcoded polynomial force surface based on fitresult from
            % make_poly_bellow_force_func()
            % p00 =  -2.387e-14;
            % p10 =      -17.33;
            % p01 =      0.4656;
            % p20 =      -22.81;
            % p11 =     -0.7803;
            % p02 =     0.02889;
            % p30 =       30.41;
            % p21 =     -0.4502;
            % p12 =    0.006453;
            % p03 =  -0.0001926;
            p00 =   5.898e-15;
            p10 =      -17.33;
            p01 =      0.4344;
            p20 =      -22.81;
            p11 =      -1.142;
            p02 =     0.04261;
            p30 =       30.41;
            p21 =     -0.6431;
            p12 =     0.01355;
            p03 =  -0.0004058;
            force = p00 + ...
                p10*strain + p01*pressure + ...
                p20*strain^2 + p11*strain*pressure + p02*pressure^2 + ...
                p30*strain^3 + p21*strain^2*pressure + p12*strain*pressure^2 + p03*pressure^3;
        end
    end
end

