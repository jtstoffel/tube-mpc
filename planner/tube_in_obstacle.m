function in_obstacle = tube_in_obstacle(tube, system, map)
    if states_in_obstacle(tube.z([1 3],:), map) % check nominal trajectory first to save time 
        in_obstacle = true;
        return
    end

    for i = 1:tube.N
        cross_section = Polyhedron('A', system.C, 'b', tube.a(:,i));
        cross_section = cross_section.plus(tube.z(:,i));
        cross_section = cross_section.projection([1 3]);
        obstacle_overlap = map.intersect(cross_section);
        if all(obstacle_overlap.isEmptySet) % no overlap between cross section and obstacles
            in_obstacle = false;
        else 
            in_obstacle = true;
            return
        end
    end
end