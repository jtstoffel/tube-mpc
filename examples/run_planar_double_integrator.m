clear; clc
system = planar_double_integrator_model;
system = preprocess_system(system, true);

system.Qa = 0.01 * eye(system.qs); % adjust elasticity 
system.x0 = [5; -2; -7; 4];

bc.initial_tube.z = [1;-2;8;-1];
bc.initial_tube.v = [0;0];
bc.initial_tube.a = ones(system.qs,1);
bc.final_tube.z = [-3;0;-2;0];
bc.final_tube.v = [0;0];
bc.final_tube.a = ones(system.qs,1);

tube = etoc(system, 30, 'etoc', bc);
simdata = sim_etoc(system, tube, bc, 10, false);
postprocess_planar_double_integrator(system, tube, simdata);