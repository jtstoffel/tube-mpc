function x = sample_state(system, xf)
    i = randi(10); % 10% goal biasing
    if i == 1
        x = xf;
    else
        x = system.x_min + (system.x_max-system.x_min).*rand(system.nx,1);
    end
end