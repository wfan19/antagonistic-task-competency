classdef JacobBellowMechanics < RodMechanicsBase

    properties
        
    end

    methods
        function obj = JacobBellowMechanics(l_0)
            arguments
                l_0 = 1;
            end
            f_force = @JacobBellowMechanics.actuator_force;
            obj@RodMechanicsBase(l_0, f_force);

            obj.e_bounds = [-0.167, 0.6680];
            obj.a_bounds = [-0.1, 92];
        end
    end

    methods(Static)
        % Derived from muscle fit data taken over summer 2023.
        function force = actuator_force(strain,pressure)
            p00 =           0;
            p10 =      0.1047;
            p01 =      -1.068;
            p20 =  -0.0007344;
            p11 =    0.005504;
            p02 =     0.00205;
            p30 =   1.939e-05;
            p21 =  -3.461e-06;
            p12 =  -6.294e-05;
            p03 =   3.081e-05;
            
            p00 =           0;
            p10 =       10.47;
            p01 =      -1.068;
            p20 =      -7.344;
            p11 =      0.5504;
            p02 =     0.00205;
            p30 =       19.39;
            p21 =    -0.03461;
            p12 =   -0.006294;
            p03 =   3.081e-05;
           
            force = p00 + ...
                p10 * strain + p01 * pressure + ...
                p20 * strain^2  + p11 * strain * pressure + p02 * pressure^2 + ...
                p30 * strain^3 + p21 * strain^2 * pressure + p12 * strain * pressure^2 + p03 * pressure^3;
            force = -force;
        end

        function K = get_stiffness_mat()
            p00 =           0;
            p10 =      0.1047;
            p01 =      -1.068;
            p20 =  -0.0007344;
            p11 =    0.005504;
            p02 =     0.00205;
            p30 =   1.939e-05;
            p21 =  -3.461e-06;
            p12 =  -6.294e-05;
            p03 =   3.081e-05;



            p00 =           0;
            p10 =       10.47;
            p01 =      -1.068;
            p20 =      -7.344;
            p11 =      0.5504;
            p02 =     0.00205;
            p30 =       19.39;
            p21 =    -0.03461;
            p12 =   -0.006294;
            p03 =   3.081e-05;

            K = [
                0   0   0   p03;
                0   0   p12 p02;
                0   p21 p11 p01;
                p30 p20 p10 p00;
            ];
        end
    end
end

