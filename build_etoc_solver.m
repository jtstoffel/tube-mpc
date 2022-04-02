function  etoc_solver = build_etoc_solver(system, N, tube_type, final_elasticity_flag)

%% Argument Check
if nargin < 4
    final_elasticity_flag = false;
elseif nargin < 3
    tube_type = 'etoc';
    final_elasticity_flag = false;
end

if not(any(strcmp(tube_type,{'etoc','htoc','fixed'})))
    tube_type = 'etoc';
end


%% Solver Settings
opts = sdpsettings('solver','mosek','verbose',0, 'beeponproblem', []);
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
z0 = sdpvar(nx,1);
zf = sdpvar(nx,1);
a = sdpvar(qs,N);
a0 = sdpvar(qs,1);
af = sdpvar(qs,1);


%% Objective and Constraints
obj = 0;
cons = [];
cons = [cons, -a <= 0]; % positive vector

% Initial Constraints
cons = [cons, z(:,1) == z0, a(:,1) == a0];

% Terminal Constraints
cons = [cons, z(:,N) == zf];
if final_elasticity_flag
    cons = [cons, a(:,N) == af];
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
disp('Building optimization solver ...')
fprintf('%i time step tube solver\n',N)
tic
etoc_solver = optimizer(cons,obj,opts, {z0,a0,zf,af}, {z,a,v,obj} );
toc

