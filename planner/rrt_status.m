function rrt_status(iteration, imax, G, obstacle_samples, tube_failures)
    clc
    disp('-------------------------------------------------')
    disp('Running Tube-to-Tube RRT Motion Planning ...')
    fprintf('Iteration: %i (%3.1f%%) \n', iteration, 100 * iteration/imax)
    fprintf('Nodes added: %i \n', G.numnodes)
    fprintf('Invalid state samples: %i  (%3.1f%%)\n', obstacle_samples, 100 * obstacle_samples/iteration)
    fprintf('Invalid tubes solved: %i (%3.1f%%) \n', tube_failures, 100 * tube_failures/(iteration-obstacle_samples))
end