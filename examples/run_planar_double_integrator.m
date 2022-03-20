clear; clc
system = planar_double_integrator_model;
system = preprocess_system(system, true);

system.Qa = 1 * eye(system.qs); % adjust elasticity 
system.x0 = [5; -2; -7; 4];

tube = etoc(system, 25);
simdata = sim_etoc(system, tube, 10, false);
postprocess_planar_double_integrator(system, tube, simdata);