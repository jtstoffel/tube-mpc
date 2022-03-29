clear; clc
system = planar_double_integrator_model;
system = preprocess_system(system, true);

system.Qa = 0.01 * eye(system.qs); % adjust elasticity 

bc.initial_tube.z = [1;-2;8;-1];
bc.initial_tube.a = ones(system.qs,1);
bc.initial_state = [];
bc.final_tube.z = [-3;-1;-2;-4];
bc.final_tube.a = ones(system.qs,1);
bc.final_state = [];

tube = etoc(system, 20, bc, 'htoc');
simdata = sim_etoc(system, tube, bc, 10, false);
postprocess_planar_double_integrator(system, tube, simdata);