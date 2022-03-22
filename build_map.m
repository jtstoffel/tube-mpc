function map = build_map()

square = Polyhedron('A', normalized_inequality_constraint([-1;-1],[1;1]), 'b', [1;1;1;1]);

obs1 = square.plus([5;5]);
obs2 = square.plus([3;1]);
obs3 = square.plus([7;8]);

map = [obs1, obs2, obs3];

