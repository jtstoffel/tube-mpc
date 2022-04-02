rng(1);
close all

%% Setup
system = planar_double_integrator_model; % only supported model currently
system = preprocess_system(system, true);
system.Qa = 0.1 * eye(system.qs); % adjust elasticity 

% Initial and final states
z0 = [0;0;0;0];
zf = [8;0;5;0];

% Iterations
imax = 100000;
stop_on_goal = false;
tube_failures = 0;
unused_samples = 0;

%% Build ETOC solver
solver_step_size = [3 5 7];
etoc_solvers =  {};
for i = 1:length(solver_step_size)
    etoc_solvers{i} = build_etoc_solver(system, solver_step_size(i),  'etoc', false);
end

%% Initialize graph
G = digraph;
G = addnode(G, {'1'});
G.Nodes.state = {z0};
G.Nodes.elasticity = {ones(system.qs,1)};

%% Load obstacle data
map = build_map();

%% RRT
tic
for i = 2:imax
    rrt_status(i, imax, G, unused_samples, tube_failures);
    new_state = sample_state(system, zf);
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
        [tube, cost, valid] = steer_etoc(system, nearest_state, nearest_elasticity, new_state, map, etoc_solvers);
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
toc

%% Global Trajectory 
all_states = cell2mat(G.Nodes.state');
distances = vecnorm(all_states - zf);
[minimum_distance, nearest_node_id] = min(distances);
[path_nodes, total_cost, path_edges] = shortestpath(G,1, nearest_node_id);
global_trajectory = cell2mat(G.Edges.tube(path_edges)');
zs = [];
vs = [];
costs = [];
for i = 1:length(path_edges)
    zs = [zs, global_trajectory(i).z];
    vs = [vs, global_trajectory(i).v];
    costs = [costs, global_trajectory(i).cost];
end

%% Trajectory Plot
figure(1)
cmap = cool;
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
scatter(z0(1), z0(3), 'ro',LineWidth=3)
scatter(zf(1), zf(3), 'go',LineWidth=3)
set(gca,'FontSize',18)


%% State and Control Plots
figure(2)
sgtitle('Nominal State Trajectories','FontSize',18)
subplot(2,2,1)
stairs(zs(1,:),LineWidth=2)
ylabel('z1')
xlabel('Time step')
set(gca,'FontSize',18)

subplot(2,2,2)
stairs(zs(2,:),LineWidth=2)
ylabel('z2')
xlabel('Time step')
set(gca,'FontSize',18)

subplot(2,2,3)
stairs(zs(3,:),LineWidth=2)
ylabel('z3')
xlabel('Time step')
set(gca,'FontSize',18)

subplot(2,2,4)
stairs(zs(4,:),LineWidth=2)
ylabel('z4')
xlabel('Time step')
set(gca,'FontSize',18)

figure(3)
vs(isnan(vs)) = 0;
sgtitle('Nominal Control Input','FontSize',18)
subplot(2,1,1)
stairs(vs(1,:),LineWidth=2)
ylabel('v1')
xlabel('Time step')
set(gca,'FontSize',18)

subplot(2,1,2)
stairs(vs(2,:),LineWidth=2)
ylabel('v2')
xlabel('Time step')
set(gca,'FontSize',18)

%% Cost Plot
figure(6)
stairs(costs,LineWidth=2)
ylabel('Cost')
xlabel('Trajectory Tube Section')
set(gca,'xtick',1:path_edges)
set(gca,'FontSize',18)

%% Aux Functions
function x = sample_state(system, xf)
    i = randi(10);
    if i == 1
        x = xf;
    else
        x = system.x_min + (system.x_max-system.x_min).*rand(system.nx,1);
    end
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

function [tube, cost, valid] = steer_etoc(system, z1, a1, z2, map, etoc_solvers)
    tube = [];
    cost = [];
    valid = false;
    for i = 1:length(etoc_solvers)
        etoc_solver = etoc_solvers{i};
        [sol, err] = etoc_solver(z1,a1,z2,ones(system.qs,1));
        if not(err)
            tube.z = sol{1};
            tube.a = sol{2};
            tube.v = sol{3};
            tube.cost = sol{4};
            cost = tube.cost;
            [~,N] = size(sol{1});
            tube.N = N;
            tube.success = true;
            if not(tube_in_obstacle(tube, system, map))
                valid = true;
                break
            end
        end
    end
end

function rrt_status(iteration, imax, G, obstacle_samples, tube_failures)
    clc
    disp('-------------------------------------------------')
    disp('Running Tube-to-Tube RRT Motion Planning ...')
    fprintf('Iteration: %i (%3.1f%%) \n', iteration, 100 * iteration/imax)
    fprintf('Nodes added: %i \n', G.numnodes)
    fprintf('Invalid state samples: %i  (%3.1f%%)\n', obstacle_samples, 100 * obstacle_samples/iteration)
    fprintf('Invalid tubes solved: %i (%3.1f%%) \n', tube_failures, 100 * tube_failures/(iteration-obstacle_samples))
end