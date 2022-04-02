function test()

%% MPT3
P = Polyhedron();

%% YALMIP/MOSEK
yalmiptest('mosek')

disp('Finished setup test')

end

