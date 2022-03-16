clear; clc
system = htmpc_example_model;
system = preprocess_system(system, true);
system.Qa = 1 * eye(system.qs); % adjust elasticity 
tube = etoc(system, 25);
simdata = sim_etoc(system, tube, 50, false);
postprocess_htmpc_example(system, tube, simdata);