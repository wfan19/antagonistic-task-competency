classdef Pose2 < GeneralLinearGroup
    %Pose2: Manages a 2D homogeneous transform matrix, which is an element
    %of SE(2), the group of 2D Euclidean transformations (isometries).

    properties (Constant)
        algebra = Twist2
        dof = 3
        mat_size = [3, 3]
    end
    
    methods
        function obj = Pose2()
        end
    end
    
    methods(Static)
        function Pose2_out = hat(v_Pose2)
            x = v_Pose2(1);
            y = v_Pose2(2);
            theta = v_Pose2(3);

            Pose2_out = eye(3, class([x, y, theta]));
            Pose2_out(1:2, 1:2) = Rot2.hat(theta);
            Pose2_out(1:2, 3) = [x; y];
        end
        
        function v_Pose2_out = vee(Pose2_in)
            v_Pose2_out = zeros(3, 1);
            v_Pose2_out(1:2) = Pose2_in(1:2, 3);
            v_Pose2_out(3) = atan2(Pose2_in(2, 1), Pose2_in(1, 1));
        end

        
        function Twist2_out = logm(mat_in)
            Twist2_out = Twist2(logm(mat_in));
        end
        
        function adj_out = adjoint(Pose2_in)
            R = Pose2_in(1:2, 1:2);
            t = Pose2_in(1:2, 3);
            
            adj_out = eye(3, class([t(:); R(:)]));
            adj_out(1:2, 1:2) = R;
            adj_out(1:2, 3) = -Omega2.hat(1) * t(:);
        end

        function t_out = translation(Pose2_in)
            t_out = Pose2_in(1:2, 3);
        end

        function R_out = rotation(Pose2_in)
            R_out = Pose2_in(1:2, 1:2);
        end

        function t_out = v_translation(v_Pose2_in)
            t_out = v_Pose2_in(1:2, :);
        end

        function R_out = v_rotation(v_Pose2_in)
            R_out = v_Pose2_in(3, :);
        end

        function TeLg = left_lifted_action(Pose2_in)
            % Left lifted action on SE2
            % AKA the jacobian of a left-action delta_g * g evaluated at g
            % Formula taken from Geometric Mechanics ver 2022/12/4 pg 151
            TeLg = eye(3, class(Pose2_in));
            TeLg(1:2, 1:2) = Pose2_in(1:2, 1:2);
        end

        function TeRg = right_lifted_action(Pose2_in)
            % Right lifted action on SE2
            % Aka the jacobian of a right-action g * delta_g evaluated at g
            % Formula taken from GM ver 2022/12/4 pg 167
            TeRg = eye(3, class(Pose2_in));
            TeRg(1, 3) = -Pose2_in(2, 3);
            TeRg(2, 3) = Pose2_in(1, 3);
        end
    end
end

