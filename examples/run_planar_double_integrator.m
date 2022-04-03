clear; clc
system = planar_double_integrator_model;
system = preprocess_system(system, true);

system.Qa = 0.1 * eye(system.qs); % adjust elasticity 

bc.initial_tube.z = [1;-1;6;-1];
bc.initial_tube.a = ones(system.qs,1);
bc.initial_state = [];
bc.final_tube.z = [2;-1;5;-1];
bc.final_tube.a = ones(system.qs,1);
bc.final_state = [];

tube = etoc(system, 10, bc, 'etoc');
simdata = sim_etoc(system, tube, bc, 10, false);
plot_4D_etoc(system, tube, simdata);