clear; clc
%% System Data
system = planar_double_integrator_model;
system = preprocess_system(system, true);
system.Qa = 1 * eye(system.qs);

%% Initial and final states
z0 = [0;0;0;0];
zf = [8;0;5;0];

%% Build map
map = build_map;

%% Planner settings
% method = 'rrt';
method = 'rrtstar';
tube_type = 'etoc';
max_iterations = 5000;
stop_on_goal = false;
solver_tube_lengths = [3 5];

%% Run planner
G = rrtstar_etoc(z0, zf, map, method, tube_type, max_iterations, stop_on_goal, solver_tube_lengths);

%% Optimal Trajectory 
global_trajectory = optimal_path_to_state(G, zf);

%% Plot results
plot_rrtstar_etoc(system, map, global_trajectory, z0, zf)