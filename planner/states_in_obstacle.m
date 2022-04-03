function in_obstacle = states_in_obstacle(states, map)
    in_obstacle = any(any(map.contains(states)));
end