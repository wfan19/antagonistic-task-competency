classdef Twist3 < GeneralLinearAlgebra
    %Twist3: Manages a 3D twist-matrix, which is an element of se(3), the
    %Lie algebra of SE(3).
    
    properties (Constant)
        dof = 6
        mat_size = [4, 4]
    end
    
    methods
        function obj = Twist3()
        end
    end
    
    methods (Static)
        %% se3 matrix constructor
        function Twist3_out = hat(v_Twist3)
            v = v_Twist3(1:3); 
            omega = v_Twist3(4:6);
            Twist3_out = zeros(4, class(v_Twist3));

            mat_omega = Omega3.hat(omega);

            Twist3_out(1:3, 1:3) = mat_omega;
            Twist3_out(1:3, 4) = v;            
        end
        
        %% SE3 Vee: Maps from matrix representation to vector representation
        function v_Twist3_out = vee(mat_Twist3)
            v_vel = mat_Twist3(1:3, 4);

            mat_Omega3 = mat_Twist3(1:3, 1:3);
            v_omega = [mat_Omega3(3, 2);
                        -mat_Omega3(3, 1);
                        mat_Omega3(2, 1)
            ];

            v_Twist3_out = [v_vel; v_omega];
        end
        
        %% SE3 Expm
        function SE3_out = expm(v_Twist3)
            % Analytic form of the SE3 exponentail map, from here: https://arxiv.org/pdf/1812.01537.pdf
            % Extract linear/angular components from a single vector
            if size(v_Twist3) == Twist3.mat_size
                v_Twist3 = Twist3.vee(v_Twist3);
            end
            
            vel = v_Twist3(1:3);
            omega = v_Twist3(4:6);

            omega = omega(:);
            vel = vel(:);

            class_string = class([omega(:), vel(:)]);
            SE3_out = eye(4, class_string);

            theta = norm(omega);

            omega_hat = Omega3.hat(omega);
            if class(theta) == "sym"
                    V = eye(3, class_string) + (1 - cos(theta)) / theta^2 * omeag_hat + ...
                        (theta - sin(theta))/theta^3 * omega_hat^2;
            else
                if round(theta, 6) > 0 % Separate because "round(sym) > 0" throws errors
                    V = eye(3, class_string) + (1 - cos(theta)) / theta^2 * omega_hat + ...
                        (theta - sin(theta))/theta^3 * omega_hat^2;
                else
                    % For small angles this is what the coefficients converge to, not
                    % infinity
                    % Can be calculated via l'Hopitals
                    V = eye(3, class_string) + 0.5 * omega_hat + (1/6) * omega_hat;
                end 
            end

            SE3_out(1:3, 1:3) = Omega3.expm(omega);
            SE3_out(1:3, 4) = V * vel;
        end

        function v_out = translation(Twist3_in)
            v_Twist3_in = Twist3.vee(Twist3_in);
            v_out = Twist3.v_translation(v_Twist3_in);
        end

        function omega_out = rotation(Twist3_in)
            v_Twist3_in = Twist3.vee(Twist3_in);
            omega_out = Twist3.v_rotation(v_Twist3_in);
        end

        function v_out = v_translation(v_Twist3_in)
            v_out = v_Twist3_in(1:3);
            v_out = v_out(:);
        end

        function omega_out = v_rotation(v_Twist3_in)
            omega_out = v_Twist3_in(4:6);
            omega_out = omega_out(:);
        end
    end
end

