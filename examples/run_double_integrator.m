clear; clc
system = double_integrator_model;
system = preprocess_system(system, true);
system.Qa = 1 * eye(system.qs); % adjust elasticity 
tube = etoc(system, 25);
simdata = sim_etoc(system, tube, 50, false);
postprocess_double_integrator(system, tube, simdata);