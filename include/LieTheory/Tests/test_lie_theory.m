function tests = test_lie_theory
tests = functiontests(localfunctions);
end

function test_expm_all(testCase)
    function test_expm(testCase, group)
        mat_algebra = group.algebra.hat(1:group.dof);

        fprintf("Testing %s\n", class(group.algebra));
        verifyEqual(testCase, group.algebra.expm(mat_algebra), expm(double(mat_algebra)), 'abstol', 1e-8);
    end
    
    groups = {SO2, SO3, SE2, SE3};
    for i = 1 : length(groups)
        test_expm(testCase, groups{i})
    end
end
    
function test_hat_so3(testCase)
    v_omega = [1 2 3];
    mat_omega = zeros(3, 3);
    
    mat_omega(3, 2) = v_omega(1);
    mat_omega(3, 1) = -v_omega(2);
    mat_omega(2, 1) = v_omega(3);

    mat_omega = mat_omega - mat_omega';
    verifyEqual(testCase, so3.hat(v_omega), mat_omega);
end