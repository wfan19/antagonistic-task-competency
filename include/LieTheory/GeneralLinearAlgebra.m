classdef GeneralLinearAlgebra
    %gl_n: Base class for Lie algebras
    
    properties(Abstract, Constant)
        dof
        mat_size;
    end
    
    methods
        function obj = GeneralLinearAlgebra()
        end
    end
    
    methods(Static, Abstract) 
        hat(v_gl_n)
        
        vee(mat_gl_n)
        
        expm(v_gl_n)
    end
end

