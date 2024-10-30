classdef AnnealedGinaMuscleMechanics < RodMechanicsBase
    
    methods
        function obj = AnnealedGinaMuscleMechanics(l_0)
            arguments
                l_0 = 1;
            end
            strain_offset = -0.1;
            %neutral_pressure = fsolve(@(pressure) GinaMuscleMechanics.actuatorForce_key(strain_offset, pressure), 50);
            neutral_pressure = 26.7384;
            f_force = @(strain, pressure) GinaMuscleMechanics.actuatorForce_key(strain + strain_offset, pressure + neutral_pressure);
            obj@RodMechanicsBase(l_0, f_force);

            obj.e_bounds = [-0.5, 0.05] - strain_offset;
            obj.a_bounds = [0, 100];
        end
    end
end

