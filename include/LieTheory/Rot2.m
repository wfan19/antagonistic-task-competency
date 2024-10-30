classdef Rot2 < GeneralLinearGroup
    % Rot2: Manages a rotation matrix, which is an element of SO(2) - the
    % group of 2D rotations.
        
    properties (Constant)
        algebra = Omega2
        dof = 1
        mat_size = [2, 2]
    end
    
    methods
        function obj = Rot2()
        end
    end
    
    methods(Static)
        function SO2_out = hat(theta)
             SO2_out = [cos(theta), -sin(theta); sin(theta), cos(theta)];
             if class(theta) ~= "sym" % Round if we are constructing a numeric matrix
                SO2_out = round(SO2_out, 16); % 16 places should be sufficient?
             end
        end
        
        function out = vee(in)
            out = 0;
        end
        
        function out = logm(mat_in)
            out = logm(mat_in);
        end
        
        function out = adjoint(in)
            out = in;
        end
    end
end