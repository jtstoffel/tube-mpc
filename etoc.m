function tube = etoc(system, N, tube_type, bc)
%% Elastic Tube Optimal Control Example
% Josh Stoffel
disp('-------------------------------------------------')
disp('Starting Elastic Tube Optimal Control Problem ...')

%% Boundary Condition Argument Check
if nargin < 3
    tube_type = 'etoc';
    bc.initial_state = system.x0;
    bc.initial_tube = [];
    bc.final_state = zeros(system.nx,1);
    bc.final_tube = [];
elseif nargin < 4
    bc.initial_state = system.x0;
    bc.initial_tube = [];
    bc.final_state = zeros(system.nx,1);
    bc.final_tube = [];
end

%% Solver Settings
opts = sdpsettings('solver','mosek','verbose',0);
tol = 1e-8;

%% System Data
A = system.A;
B = system.B;
nx = system.nx;
nu = system.nu;
qs = system.qs;
qx = system.qx;
qu = system.qu;
C = system.C;
d = system.d;
L1 = system.L1;
M1 = system.M1;
T1 = system.T1;
G = system.G;
H = system.H;
Qz = system.Qz;
Qv = system.Qv;
Qa = system.Qa;
Pz = system.Pz;
Pa = system.Pa;

%% Decision Variables
v = sdpvar(nu,N);
z = sdpvar(nx,N);
a = sdpvar(qs,N);

%% Objective and Constraints
obj = 0;
cons = [];
cons = [cons, -a <= 0]; % positive vector

% Initial Constraints
if ~isempty(bc.initial_tube) 
    z0 = bc.initial_tube.z;
    v0 = bc.initial_tube.v;
    a0 = bc.initial_tube.a;
    cons = [cons, z(:,1) == z0, v(:,1) == v0, a(:,1) == a0];
elseif ~isempty(bc.initial_state) 
    x0 = bc.initial_state;
    cons = [cons, C*x0 - C*z(:,1) - a(:,1) <= 0];
else
    error('Boundary condition must include one of: initial_tube, initial_state')
end

% Terminal Constraints
if ~isempty(bc.final_tube) 
    zf = bc.final_tube.z;
    vf = bc.final_tube.v;
    af = bc.final_tube.a;
    cons = [cons, z(:,N) == zf, v(:,N) == vf, a(:,N) == af];
elseif ~isempty(bc.final_state) 
    xf = bc.final_state; zf = xf;
    cons = [cons, C*xf - C*z(:,N) - a(:,N) <= 0];
else
    error('Boundary condition must include one of: final_tube, final_state')
end

for k = 1:N-1
    % Dynamics Constraint
    cons = [cons, C*A*z(:,k) + C*B*v(:,k) + L1*a(:,k) + d - C*z(:,k+1) - a(:,k+1) <= 0];

    % State Tube Constraint
    cons = [cons, G*z(:,k) + M1*a(:,k) - ones(qx,1) <= 0];
    
    % Control Tube Constraint
    cons = [cons, H*v(:,k) + T1*a(:,k) - ones(qu,1) <= 0];
    
    % Stage Cost
    obj = obj + (z(:,k)-zf)'*Qz*(z(:,k)-zf) + v(:,k)'*Qv*v(:,k) + (a(:,k)-ones(qs,1))'*Qa*(a(:,k)-ones(qs,1)); 
end

% Terminal Cost
obj = obj + (z(:,N)-zf)'*Pz*(z(:,N)-zf) + (a(:,N)-1)'*Pa*(a(:,N)-1);

% Homothetic Tube Constraint
if strcmp(tube_type, 'htoc')
    for k = 1:N
        for i = 1:qs-1
            cons = [cons, a(i,k) == a(i+1,k)];
        end
    end
end

% Fixed Tube Constraint
if strcmp(tube_type, 'fixed')
    cons = [cons, a(:,:) == ones(qs,N)];
end

%% Optimize Tube 
disp('Solving trajectory tube ...')
tic
sol = optimize(cons,obj,opts);
tube.z = value(z);
tube.v = value(v);
tube.a = value(a);
tube.N = N;
tube.cost = value(obj);
tube.success = true;

if sol.problem ~= 0
    disp('FAILURE: Unable to solve tube trajectory')
    disp(sol)
    tube.success = false;
end

toc