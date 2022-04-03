clear; clc
system = double_integrator_model;
system = preprocess_system(system, true);
system.Qa = 1 * eye(system.qs); % adjust elasticity 

bc.initial_tube.z = [1;-2];
bc.initial_tube.a = ones(system.qs,1);
bc.initial_state = [];
bc.final_tube.z = [0;0];
bc.final_tube.a = ones(system.qs,1);
bc.final_state = [];

tube = etoc(system, 10, bc);
simdata = sim_etoc(system, tube, bc, 10, false);
plot_2D_etoc(system, tube, simdata);