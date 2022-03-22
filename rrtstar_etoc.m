rng(1);
%% Psuedo Code
% Rad = r
% G(V,E) //Graph containing edges and vertices
% For itr in range(0…n)
%     Xnew = RandomPosition()
%     If Obstacle(Xnew) == True, try again
%     Xnearest = Nearest(G(V,E),Xnew)
%     Cost(Xnew) = Distance(Xnew,Xnearest)
%     Xbest,Xneighbors = findNeighbors(G(V,E),Xnew,Rad)
%     Link = Chain(Xnew,Xbest)
%     For x’ in Xneighbors
%         If Cost(Xnew) + Distance(Xnew,x’) < Cost(x’)
%             Cost(x’) = Cost(Xnew)+Distance(Xnew,x’)
%             Parent(x’) = Xnew
%             G += {Xnew,x’}
%     G += Link 
% Return G


%% 
% Choose Model
system = planar_double_integrator_model;
system = preprocess_system(system, true);

% Near neighbor radius
radius = 1;

% Initialize graph
G = graph;
G = addnode(G, 2);
G.Nodes.pos = {[0;0]};

% Iterations
imax = 1000;

% Load obstacle data
map = build_map();

positions = [];
for i = 1:imax
    new_state = sample_state(system);
    new_pos = new_state([1 3]);
    if state_in_obstacle(new_pos, map)
        continue
    else
        positions = [positions, new_pos];
    end
end

figure(1)
X_pos = system.X.projection([1 3]);
X_pos.plot('wire',true)
hold on; axis equal
map.plot
scatter(positions(1,:), positions(2,:))


%% Functions
function x = sample_state(system)    
    x = system.x_min + (system.x_max-system.x_min).*rand(system.nx,1);
end

function cost = get_cost(x1, x2)
    cost = norm(x1-x2);
end

function in_obstacle = state_in_obstacle(state, map)
    in_obstacle = false;
    for i = 1:length(map)
        if map(i).contains(state)
            in_obstacle = true;
            break
        end
    end
end


