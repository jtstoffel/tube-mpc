function rrtstar_status(iteration, imax, G, obstacle_samples, tube_failures, nodes_rewired)
    clc
    disp('-------------------------------------------------')
    disp('Running Tube-to-Tube RRT* Motion Planning ...')
    fprintf('Iteration: %i (%3.1f%%) \n', iteration, 100 * iteration/imax)
    fprintf('Nodes added: %i \n', G.numnodes)
    fprintf('Nodes rewired: %i \n', nodes_rewired)
    fprintf('Invalid state samples: %i \n', obstacle_samples)
    fprintf('Invalid tubes solved: %i \n', tube_failures)
end