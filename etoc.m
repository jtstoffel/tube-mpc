function tube = etoc(system, N, tube_type)
%% Elastic Tube Optimal Control Example
% Josh Stoffel
disp('-------------------------------------------------')
disp('Starting Elastic Tube Optimal Control Problem ...')

if nargin < 3
    tube_type = 'etoc';
end

%% Solver Settings
opts = sdpsettings('solver','mosek','verbose',0);
tol = 1e-8;

%% System Data
A = system.A;
B = system.B;
x0 = system.x0;
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
disp('-------------------------------------------------')
disp('Solving trajectory tube ...')
tic
obj = 0;
cons = [];
cons = [cons, -a <= 0]; % positive vector
cons = [cons, C*x0 - C*z(:,1) - a(:,1) <= 0]; % initial state set constraint

for k = 1:N-1
    % Dynamics Constraint
    cons = [cons, C*A*z(:,k) + C*B*v(:,k) + L1*a(:,k) + d - C*z(:,k+1) - a(:,k+1) <= 0];

    % State Tube Constraint
    cons = [cons, G*z(:,k) + M1*a(:,k) - ones(qx,1) <= 0];
    
    % Control Tube Constraint
    cons = [cons, H*v(:,k) + T1*a(:,k) - ones(qu,1) <= 0];
    
    % Stage Cost
    obj = obj + z(:,k)'*Qz*z(:,k) + v(:,k)'*Qv*v(:,k) + (a(:,k)-ones(qs,1))'*Qa*(a(:,k)-ones(qs,1)); 
end

% Terminal Constraints
cons = [cons, z(:,N) <= 0.01 * ones(nx,1)];
cons = [cons, z(:,N) >= -0.01 * ones(nx,1)];
cons = [cons, a(:,N) == ones(qs,1)];
cons = [cons, v(:,N) == zeros(nu,1)];

% Terminal Cost
% obj = obj + z(:,N)'*Pz*z(:,N) + (a(:,N)-1)'*Pa*(a(:,N)-1);

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

%% Optimize
optimize(cons,obj,opts);
tube.z = value(z);
tube.v = value(v);
tube.a = value(a);
tube.N = N;

toc