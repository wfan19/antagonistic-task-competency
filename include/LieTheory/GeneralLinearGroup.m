classdef GeneralLinearGroup
    % GL_n: The general linear group
    % This absrtact class defines the interfaces that all Lie Group
    % subclasses will have
    
    properties(Abstract, Constant)
        algebra
        
        dof
        mat_size
    end
    
    methods
        function obj = GeneralLinearGroup()
        end
    end
    
    methods(Static, Abstract)
        hat()
        
        vee()
        
        logm()
        
        adjoint()
    end
end

