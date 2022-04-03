clear; clc
% System Data
system = planar_double_integrator_model;
system = preprocess_system(system, true);

% Initial and final states
z0 = [0;0;0;0];
zf = [8;0;5;0];

% Build map
map = build_map;

% Planner settings
method = 'rrt';
tube_type = 'etoc';
max_iterations = 10000;
stop_on_goal = true;
solver_tube_lengths = [3 5 7];

% Run planner
global_trajectory = rrtstar_etoc(z0, zf, map, method, tube_type, max_iterations, stop_on_goal, solver_tube_lengths);

% Plot results
plot_rrtstar_etoc(system, map, global_trajectory, z0, zf)