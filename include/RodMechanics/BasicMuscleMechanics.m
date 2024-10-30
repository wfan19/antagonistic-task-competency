classdef BasicMuscleMechanics < RodMechanicsBase
    properties
        l_0 = 1
        f_force = @f_force_default
    end
    
    methods
        function obj = BasicMuscleMechanics(l_0, f_force)
            arguments
                l_0 = 1;
                f_force = @(strain, pressure) 0;
            end
            obj.l_0 = l_0;       
            obj.f_force = f_force;
        end
    end

    methods(Static)
        function fitresult = make_muscle_force_func()
            fitresult = @(strain, pressure) 0;
            error("The BasicMuscleMechanics model is not yet implemented")
        end
    end
end

