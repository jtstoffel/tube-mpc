function G = normalized_inequality_constraint(x_min, x_max)

if length(x_min) ~= length(x_max)
    error('x_min and x_max vectors must be equal length');
end

n = length(x_min); % number of states
G = zeros(2*n,n);

for i = 1:n
    
    if x_min(i) < 0
        G(i,i) = 1 / x_min(i);
    elseif x_min(i) == 0
        G(i,i) = 0; 
    else
        G(i,i) = 1 / -x_min(i);
    end
    
    if x_max(i) < 0
        G(i+n,i) = 1 / -x_max(i);
    elseif x_max(i) == 0
        G(i+n,i) = 0; 
    else
        G(i+n,i) = 1 / x_max(i);
    end


end

end