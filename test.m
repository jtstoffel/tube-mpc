function test(setupTestFlag, examplesTestFlag)

if nargin < 2
    examplesTestFlag = false;
elseif nargin < 1
    setupTestFlag = true;
    examplesTestFlag = true;
end

if setupTestFlag
    % MPT3
    try
        P = Polyhedron();
    catch
        disp('SETUP TEST FAILURE: MPT3 NOT FOUND')
    end

    % YALMIP/MOSEK
    yalmiptest('mosek')
    disp('Success! Finished setup test')

elseif examplesTestFlag
    % Examples test
    try 
        run_double_integrator;
    catch
        disp('EXAMPES TEST FAILURE: run_double_integrator')
    end
    try 
        run_spring_mass_damper;
    catch
        disp('EXAMPES TEST FAILURE: run_spring_mass_damper')
    end
    try 
        run_planar_double_integrator;
    catch
        disp('EXAMPES TEST FAILURE: run_planar_double_integrator')
    end
    try 
        run_rrt_example;
    catch
        disp('EXAMPES TEST FAILURE: run_rrt_example')
    end

    close all
    disp('Success! Finished examples test')
end
end

