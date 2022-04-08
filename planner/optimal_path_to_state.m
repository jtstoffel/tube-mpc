function global_trajectory = optimal_path_to_state(G, zf)
all_states = cell2mat(G.Nodes.state');
distances = vecnorm(all_states - zf);
[~, nearest_node_id] = min(distances);
[~, ~, path_edges] = shortestpath(G,1, nearest_node_id);
global_trajectory = cell2mat(G.Edges.tube(path_edges)');
end

