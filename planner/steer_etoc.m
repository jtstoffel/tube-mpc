function [tube, cost, valid] = steer_etoc(system, z1, a1, z2, a2, map, etoc_solvers)
    tube = [];
    cost = [];
    valid = false;
    for i = 1:length(etoc_solvers) % attempt to solve in order of etoc_solvers
        etoc_solver = etoc_solvers{i};
        [sol, err] = etoc_solver(z1,a1,z2,a2);
        if not(err)
            tube.z = sol{1};
            tube.a = sol{2};
            tube.v = sol{3};
            tube.cost = sol{4};
            cost = tube.cost;
            [~,N] = size(sol{1});
            tube.N = N;
            tube.success = true;
            if not(tube_in_obstacle(tube, system, map)) % check obstacle collision
                valid = true;
                break
            end
        end
    end
end