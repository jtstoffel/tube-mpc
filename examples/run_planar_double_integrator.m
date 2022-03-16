clear; clc
system = planar_double_integrator_model;
system = preprocess_system(system, true);
tube = etoc(system, 25);
simdata = sim_etoc(system, tube, 50, false);
postprocess_planar_double_integrator(system, tube, simdata);