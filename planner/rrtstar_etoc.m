function global_trajectory = rrtstar_etoc(z0, zf, map, method, tube_type, max_iterations, stop_on_goal, solver_tube_lengths)
rng(1);

%% System data
system = planar_double_integrator_model; % only supported model currently
system = preprocess_system(system, true);
system.Qa = 0.1 * eye(system.qs); % adjust elasticity 

%% Build ETOC solver(s)
if strcmp(method, 'rrt')
    steer_etoc_solvers =  {};
    for i = 1:length(solver_tube_lengths)
        steer_etoc_solvers{i} = build_etoc_solver(system, solver_tube_lengths(i), tube_type, false);
    end
elseif strcmp(method, 'rrtstar')
    steer_etoc_solvers =  {};
    rewire_etoc_solvers =  {};
    for i = 1:length(solver_tube_lengths)
        steer_etoc_solvers{i} = build_etoc_solver(system, solver_tube_lengths(i), tube_type, false);
        rewire_etoc_solvers{i} = build_etoc_solver(system, solver_tube_lengths(i), tube_type, true);
    end
end

%% Initialize graph
G = digraph;
G = addnode(G, {'1'});
G.Nodes.state = {z0};
G.Nodes.elasticity = {ones(system.qs,1)};

%% RRT/RRT* Planner
tube_failures = 0;
unused_samples = 0;
tic
if strcmp(method, 'rrt') % RRT PLANNER
    for i = 2:max_iterations
        rrt_status(i, max_iterations, G, unused_samples, tube_failures);
        new_state = sample_state(system, zf);
        if states_in_obstacle(new_state([1 3]), map)
            unused_samples = unused_samples + 1;
        else
            all_states = cell2mat(G.Nodes.state');
            distances = vecnorm(all_states - new_state);
            [~, nearest_node_id] = min(distances);
            nearest_state = cell2mat(G.Nodes.state(nearest_node_id));
            nearest_elasticity = cell2mat(G.Nodes.elasticity(nearest_node_id));
            [tube, cost, valid] = steer_etoc(system, nearest_state, nearest_elasticity, new_state, ones(system.qs,1), map, steer_etoc_solvers);
            if valid
                new_node = table({num2str(i)}, {tube.z(:,tube.N)}, {tube.a(:,tube.N)}, 'VariableNames', {'Name' 'state' 'elasticity'});
                G = addnode(G, new_node);
                new_edge = table([G.Nodes.Name(nearest_node_id), {num2str(i)}], {tube}, cost, 'VariableNames', {'EndNodes' 'tube' 'weight'});
                G = addedge(G, new_edge);
                if norm(new_state - zf) < 0.5
                    if stop_on_goal
                        break
                    end
                end
            else
                tube_failures = tube_failures + 1;
            end
        end
    end

elseif strcmp(method, 'rrtstar') % RRT* PLANNER
    radius = 5;
    for i = 2:max_iterations
        rrt_status(i, max_iterations, G, unused_samples, tube_failures);
        new_state = sample_state(system, zf);
        if states_in_obstacle(new_state([1 3]), map)
            unused_samples = unused_samples + 1;
        else
            all_states = cell2mat(G.Nodes.state');
            distances = vecnorm(all_states - new_state);
            [minimum_distance, nearest_node_id] = min(distances);
            nearest_state = cell2mat(G.Nodes.state(nearest_node_id));
            nearest_elasticity = cell2mat(G.Nodes.elasticity(nearest_node_id));
            [tube, cost, valid] = steer_etoc(system, nearest_state, nearest_elasticity, new_state, map, steer_etoc_solvers);
            if valid
                new_node = table({num2str(i)}, {tube.z(:,tube.N)}, {tube.a(:,tube.N)}, 'VariableNames', {'Name' 'state' 'elasticity'});
                G = addnode(G, new_node);
                new_edge = table([G.Nodes.Name(nearest_node_id), {num2str(i)}], {tube}, cost, 'VariableNames', {'EndNodes' 'tube' 'weight'});
                G = addedge(G, new_edge);
                if norm(new_state - zf) < 0.5
                    if stop_on_goal
                        break
                    end
                end
            else
                tube_failures = tube_failures + 1;
            end
        end
    end
end
toc

%% Optimal Global Trajectory 
all_states = cell2mat(G.Nodes.state');
distances = vecnorm(all_states - zf);
[~, nearest_node_id] = min(distances);
[~, ~, path_edges] = shortestpath(G,1, nearest_node_id);
global_trajectory = cell2mat(G.Edges.tube(path_edges)');


end