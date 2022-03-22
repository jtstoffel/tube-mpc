function simdata = sim_etoc(system, tube, bc, runs, saveFlag)
%% Optimal Control Simulation
disp('-------------------------------------------------')
disp('Simulating perturbed runs ...')
tic

%% System Data
A = system.A;
B = system.B;
nx = system.nx;
nu = system.nu;
nw = system.nw;
w_min = system.w_min;
w_max = system.w_max;
K1 = system.K1;
C = system.C;

%% Tube Data
N = tube.N;
z = tube.z;
v = tube.v;

%% Boundary Condition Data
if ~isempty(bc.initial_tube)
    z0 = bc.initial_tube.z;
    a0 = bc.initial_tube.a;
    S = Polyhedron('A', C, 'b', a0);
    S = S.plus(z0);
    x0 = S.randomPoint;
elseif ~isempty(bc.initial_state) 
    x0 = bc.initial_state;
end

%% Simulation 
rng(1);
x = zeros(nx,N);
x(:,1) = x0;
xs = zeros(nx, N, runs);
u = zeros(nu, N);
us = zeros(nu, N, runs);
w = zeros(nw, N);
ws = zeros(nw, N, runs);

for i = 1:runs
    for k = 1:N-1
        u(:,k) = v(:,k) + K1*(x(:,k)-z(:,k));
%         u(:,k) = v(:,k);
        w(:,k) = (w_max-w_min).*rand(nw,1) + w_min;
        x(:,k+1) = A*x(:,k) + B*u(:,k) + w(:,k);      
    end
    xs(:,:,i) = x;
    us(:,:,i) = u;
    ws(:,:,i) = w;
end
toc

%% Save Output
simdata.xs = xs;
simdata.us = us;
simdata.ws = ws;
simdata.runs = runs;

if saveFlag
    now = datetime;
    now.Format = 'yyyy-MM-dd-HH-mm-ss';
    fn = ['./data/run_', system.name, '_', string(now)];
    save(fn, 'system', 'tube', 'simdata')
end

