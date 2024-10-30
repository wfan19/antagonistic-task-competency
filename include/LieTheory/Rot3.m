classdef Rot3 < GeneralLinearGroup
    % Rot3: Manages a 3D rotation matrix, which is an element of SO(3), the
    % group of 3D rotations.
    
    properties (Constant)
        algebra = Omega3
        dof = 6
        mat_size = [4, 4]
    end
    
    methods
        function obj = Rot3()
        end
    end
    
    methods(Static)
        % Not really implemented - use Matlab's quat2rotm or eul2rotm instead.
        function mat_out = hat(mat_R)
            mat_out = mat_R;
            if strcmp(class(mat_R), "double")
                mat_out = Rot3(mat_out);
            end
        end
        
        function v_SO3_out = vee(mat_SO3)
            v_SO3_out = [1; 0; 0]; % Not implemented - use rotm2eul instead?
        end

        function so3_out = logm(mat_in)
            % TODO: Implement analytic version if ever needed
            so3_out = so3(logm(mat_in));
        end
        
        function out = adjoint()
            % TODO: Implement if needed
        end
    end
end