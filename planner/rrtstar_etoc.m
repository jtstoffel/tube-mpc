function G = rrtstar_etoc(z0, zf, map, method, tube_type, max_iterations, stop_on_goal, solver_tube_lengths)
rng(3);

%% System data
system = planar_double_integrator_model; % only supported model currently
system = preprocess_system(system, true);

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
        rewire_etoc_solvers{i} = build_etoc_solver(system, solver_tube_lengths(i), tube_type, true); % final tube cross section must match for rewiring
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
nodes_rewired = 0;
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
                figure(10)
                G.plot
                if norm(new_state - zf) < 0.5 && stop_on_goal
                    break
                end
            else
                tube_failures = tube_failures + 1;
            end
        end
    end

elseif strcmp(method, 'rrtstar') % RRT* PLANNER
    gamma = 5;
    for i = 2:max_iterations
        rrtstar_status(i, max_iterations, G, unused_samples, tube_failures, nodes_rewired);
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
                % Add new node using nearest node
                new_node_state = tube.z(:,tube.N);
                new_node_elasticity = tube.a(:,tube.N);
                new_node = table({num2str(i)}, {new_node_state}, {new_node_elasticity}, 'VariableNames', {'Name' 'state' 'elasticity'});
                G = addnode(G, new_node);
                new_edge = table([G.Nodes.Name(nearest_node_id), {num2str(i)}], {tube}, cost, 'VariableNames', {'EndNodes' 'tube' 'weight'});
                G = addedge(G, new_edge);
                figure(10)
                G.plot

                % Rewire neighboring nodes of new node
                n = G.numnodes; 
                d = 4;
                radius = gamma*((log(n)/n)^(1/d)); 
                neighbor_ids = find(distances < radius);
                if ~isempty(neighbor_ids) % check if there are near neighbor nodes
                    for j = 1:length(neighbor_ids) % for each node
                        if neighbor_ids(j) == 1
                            continue % do not rewire root node
                        elseif neighbor_ids(j) == nearest_node_id
                            continue % nearest already connected
                        end
                        near_state = cell2mat(G.Nodes.state(neighbor_ids(j)));
                        near_elasticity = cell2mat(G.Nodes.elasticity(neighbor_ids(j)));
                        [tube_rewire, cost_rewire, valid_rewire] = steer_etoc(system, new_node_state, new_node_elasticity, near_state, near_elasticity, map, rewire_etoc_solvers);

                        if valid_rewire % check if rewire is valid 
                            [~, ~, path_edges] = shortestpath(G,1, neighbor_ids(j)); % shortest path from start to neighbor node
                            if ~isempty(path_edges)
                                original_total_cost = sum(G.Edges.weight(path_edges)); % total cost from start to neighbor node
                                Gnew = rmedge(G,path_edges(end)); % remove last segment connected to neighbor node
                                if ~isdag(Gnew)
                                    keyboard
                                end
                                rewire_edge = table([{num2str(i)}, Gnew.Nodes.Name(neighbor_ids(j))], {tube_rewire}, cost_rewire, 'VariableNames', {'EndNodes' 'tube' 'weight'});
                                Gnew = addedge(Gnew, rewire_edge); % add edge from newly added node to neighbor node
                                [~, ~, path_edges] = shortestpath(Gnew,1, neighbor_ids(j)); % shortest path from start to rewired neighbor node
                                if ~isempty(path_edges) % check removal of old edge and addition of new edge still allows valid path from start
                                    rewire_total_cost = sum(Gnew.Edges.weight(path_edges)); % total cost from start to rewired neighbor node
                                    if rewire_total_cost < original_total_cost 
                                        G = Gnew; % if cost using rewired node is less, keep changed edges
                                        nodes_rewired = nodes_rewired + 1;
                                        rrtstar_status(i, max_iterations, G, unused_samples, tube_failures, nodes_rewired);
                                    end
                                end
                            end
                        else
                            tube_failures = tube_failures + 1;
                        end
                    end
                end

                if norm(new_state - zf) < 0.5 && stop_on_goal
                    break
                end

            else
                tube_failures = tube_failures + 1;
            end
        end
    end
end
toc

end