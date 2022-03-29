rng(1);
close all

%% Setup
% Choose Model
system = planar_double_integrator_model;
system = preprocess_system(system, true);

% Initial and final states
x0 = [0;0;0;0];
xf = [8;0;5;0];

% Near neighbor radius
radius = 10;

% Iterations
imax = 4000;
stop_on_goal = false;
tube_failures = 0;
unused_samples = 0;

%% Initialize graph
G = digraph;
G = addnode(G, {'1'});
G.Nodes.state = {x0};
G.Nodes.elasticity = {ones(system.qs,1)};

%% Load obstacle data
map = build_map();

%% RRT
tic
for i = 2:imax
    rrt_status(i, imax, G, unused_samples, tube_failures);
    new_state = sample_state(system, xf);
    if states_in_obstacle(new_state([1 3]), map)
        unused_samples = unused_samples + 1;
    else        
        all_states = cell2mat(G.Nodes.state');
        distances = vecnorm(all_states - new_state);
        [minimum_distance, nearest_node_id] = min(distances);
%         if minimum_distance < radius
%             unused_samples = unused_samples + 1;
%             continue
%         end
        nearest_state = cell2mat(G.Nodes.state(nearest_node_id));
        nearest_elasticity = cell2mat(G.Nodes.elasticity(nearest_node_id));
        [tube, cost, valid] = steer_etoc(system, nearest_state, nearest_elasticity, new_state, map);
        if valid
            new_node = table({num2str(i)}, {tube.z(:,tube.N)}, {tube.a(:,tube.N)}, 'VariableNames', {'Name' 'state' 'elasticity'});
            G = addnode(G, new_node);
            new_edge = table([G.Nodes.Name(nearest_node_id), {num2str(i)}], {tube}, cost, 'VariableNames', {'EndNodes' 'tube' 'weight'});
            G = addedge(G, new_edge);
            if norm(new_state - xf) < 0.5
                if stop_on_goal
                    break
                end
            end
        else
            tube_failures = tube_failures + 1;
        end
    end
end
toc
%% Plotting

if stop_on_goal
    [path_nodes, total_cost, path_edges] = shortestpath(G,1, G.numnodes);
else 
    all_states = cell2mat(G.Nodes.state');
    distances = vecnorm(all_states - xf);
    [minimum_distance, nearest_node_id] = min(distances);
    [path_nodes, total_cost, path_edges] = shortestpath(G,1, nearest_node_id);
end
global_trajectory = cell2mat(G.Edges.tube(path_edges)');

figure(1)
cmap = bone;
X_pos = system.X.projection([1 3]);
X_pos.plot('wire',true)
hold on; axis equal
map.plot('color', 'blue')
for i = 1:length(path_edges)
    color = cmap(randi(256),:);
    z = global_trajectory(i).z;
    v = global_trajectory(i).v;
    a = global_trajectory(i).a;
    for k = 1:global_trajectory(i).N
        Sa = Polyhedron('A', system.C, 'b', a(:,k));
        Sa = Sa.plus(z(:,k));
        Sa = Sa.projection([1 3]);
        Sa.minHRep;
        Sa.plot('color', color)
    end
    plot(z(1,:),z(3,:),'m',LineWidth=3)
end


figure(2)
hold on; grid on; axis equal
for i = 1:length(path_edges)
    z = global_trajectory(i).z;
    plot(z(2,:),z(4,:),LineWidth=2)
end


%% Functions
function x = sample_state(system, xf)
    i = randi(10);
    if i == 1
        x = xf;
    else
        x = system.x_min + (system.x_max-system.x_min).*rand(system.nx,1);
    end
    
end

function cost = get_cost(x1, x2)
    cost = norm(x1-x2);
end



function in_obstacle = states_in_obstacle(states, map)
    in_obstacle = any(any(map.contains(states)));
end



function in_obstacle = tube_in_obstacle(tube, system, map)
    if states_in_obstacle(tube.z([1 3],:), map)
        in_obstacle = true;
        return
    end

    for i = 1:tube.N
        cross_section = Polyhedron('A', system.C, 'b', tube.a(:,i));
        cross_section = cross_section.plus(tube.z(:,i));
        cross_section = cross_section.projection([1 3]);
        obstacle_overlap = map.intersect(cross_section);
        if all(obstacle_overlap.isEmptySet)
            in_obstacle = false;
        else 
            in_obstacle = true;
            return
        end
    end
end



function [tube, cost, valid] = steer_etoc(system, z1, a1, z2, map)
    bc.initial_tube.z = z1;
    bc.initial_tube.a = a1;
    bc.initial_state = [];
    bc.final_tube.z = z2;
    bc.final_tube.a = [];
    bc.final_state = [];
    tube = etoc(system, 8, bc, 'fixed', true);
    cost = tube.cost;

    valid = false;
    if tube.success
        if not(tube_in_obstacle(tube, system, map))
            valid = true;
        end    
    end
end


function rrt_status(iteration, imax, G, obstacle_samples, tube_failures)
    clc
    
    disp('-------------------------------------------------')
    disp('Running Tube-to-Tube RRT Motion Planning ...')
    fprintf('Iteration: %i (%3.1f%%) \n', iteration, 100 * iteration/imax)
    fprintf('Nodes added: %i \n', G.numnodes)
    fprintf('Edges added: %i \n', G.numedges)
    fprintf('Invalid state samples: %i  (%3.1f%%)\n', obstacle_samples, 100 * obstacle_samples/iteration)
    fprintf('Invalid tubes solved: %i (%3.1f%%) \n', tube_failures, 100 * tube_failures/(iteration-obstacle_samples))
end


% function [trajectory, cost, valid] = steer(x1, x2, map)
%     n = length(x1);
%     npoints = 100;
%     trajectory = zeros(n,npoints);
%     for i = 1:n
%         trajectory(i,:) = linspace(x1(i), x2(i), npoints);
%     end
%     valid = not(states_in_obstacle(trajectory, map));
%     cost = get_cost(x1, x2);
% end


