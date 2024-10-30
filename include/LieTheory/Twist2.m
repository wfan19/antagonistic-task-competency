classdef Twist2 < GeneralLinearAlgebra
    %Twist2: Manages a 2D twist matrix, which is an element of se(2), the
    %Lie algebra of SE(2).
    
    properties (Constant)
        dof = 3
        mat_size = [3, 3]
    end
    
    methods
        function obj = Twist2()
        end
    end
    
    methods (Static)
        function Twist2_out = hat(v_Twist2)
            mat_skew_sym = Omega2.hat(v_Twist2(3));
            Twist2_out = zeros(3, class(v_Twist2));
            Twist2_out(1:2, 1:2) = mat_skew_sym;
            Twist2_out(1:2, 3) = v_Twist2(1:2);
        end
        
        function v_Twist2_out = vee(Twist2_in)
            v_Twist2_out = zeros(3, 1, class(Twist2_in));

            v_Twist2_out(1:2) = Twist2_in(1:2, 3);
            v_Twist2_out(3) = Twist2_in(2, 1);
        end
        
        function Pose2_out = expm(v_Twist2)
            % Analytic SE2 exponential map
            % - Supports both numerical and symbolic calculations
            % - If any input contains a symbolic object it will return a symbolic
            % matrix
            
            % Check if input was a matrix
            if numel(v_Twist2) ~= 3
                v_Twist2 = Twist2.vee(v_Twist2);
            end
            
            vx = v_Twist2(1);
            vy = v_Twist2(2);
            omega = v_Twist2(3);

            skew_sym_1 = Omega2.hat(1);
            if omega ~= 0
                V = (1/omega) * (sin(omega)*eye(2) + ((1-cos(omega))*skew_sym_1));
            else
                V = eye(2, class(v_Twist2));
            end

            R = Rot2.hat(omega);

            Pose2_out = eye(3, class(v_Twist2));
            Pose2_out(1:2, 1:2) = R;
            Pose2_out(1:2, 3) = V * [vx; vy];
        end

        function v_out = translation(Twist2_in)
            v_out = Twist2_in(1:2, 3);
        end

        function omega_out = rotation(Twist2_in)
            omega_out = Twist2_in(2, 1);
        end

        function v_out = v_translation(v_Twist2_in)
            v_out = v_Twist2_in(1:2);
            v_out = v_out(:);
        end

        function omega_out = v_rotation(v_Twist2_in)
            omega_out = v_Twist2_in(3);
            omega_out = omega_out(:);
        end

        function ad_out = adjoint(v_Twist2_in)
            ad_out = zeros(3, 3, class(v_Twist2_in));
            ad_out(2, 1) = v_Twist2_in(3);
            ad_out(1, 2) = -v_Twist2_in(3);
            ad_out(1, 3) = v_Twist2_in(2);
            ad_out(2, 3) = -v_Twist2_in(1);
        end
    end
end

