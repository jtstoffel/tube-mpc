%% Elastic Tube Optimal Control Example
% Josh Stoffel
disp('-------------------------------------------------')
disp('Starting Elastic Tube Optimal Control Problem ...')

%% Solver Settings
opts = sdpsettings('solver','mosek','verbose',0);
tol = 1e-8;

%% State Space System
if ~exist('system','var')
    system = htmpc_example_model();
end

% system = htmpc_example_model();
% system = lane_keeping_model();
% system = double_integrator_model();
% system = spring_mass_damper_model();

disp(system.name)
A = system.A;
B = system.B;
x0 = system.x0;

if ~exist('N','var')
    N = 50;
end

runs = 5;
[nx, nu] = size(B);



%% State Constraints
x_min = system.x_min;
x_max = system.x_max;
G = normalized_inequality_constraint(x_min, x_max);
qx = length(G);
X = Polyhedron('A',G,'b',ones(qx,1));



%% Input Constraints
u_max = system.u_max;
u_min = system.u_min;
H = normalized_inequality_constraint(u_min, u_max);
qu = length(H);
U = Polyhedron('A',H,'b',ones(qu,1));


%% Disturbance Constraints
w_min = system.w_min;
w_max = system.w_max;
nw = length(w_min);
D = normalized_inequality_constraint(w_min, w_max);
qw = length(D);
W = Polyhedron('A',D,'b',ones(qw,1));


%% Weighting Matricies
Qz = eye(nx);
Qv = 0.1*eye(nu);



%% S(1), K(1), and C
K1 = -dlqr(A,B,Qz,Qv);
Ak = A+B*K1;
S1 = compute_mrpi_set(Ak,W, 0.00001);
S1.computeHRep;

C = zeros(size(S1.A));
qs = length(C);
for k = 1:length(S1.A)
   if S1.b(k) < 0
       C(k,:) = S1.A(k,:) ./ -S1.b(k)';
   else
       C(k,:) = S1.A(k,:) ./  S1.b(k)';
   end
end

Qa = 100*eye(qs);



%% d, gamma1, eta1
disp('-------------------------------------------------')
disp('Calculating Tube Approximation Parameters ...')
tic

d = W.support(C'); % Eq 3.4

gamma1 = S1.support(G'); % Eq 3.4

K1S1 = K1*S1;
eta1 = K1S1.support(H');




%% L(1), M(1), T(1)
L1 = sdpvar(qs,qs,'full');
cons = [L1*ones(qs,1) == 1-d, L1>=0, L1*C == C*Ak];
obj = 0;
for k = 1:qs
    for j = 1:qs
        obj = obj + L1(k,j)^2;
    end
end
optimize(cons,obj)
L1 = value(L1);

M1 = sdpvar(qx,qs,'full');
cons = [M1*ones(qs,1) == gamma1, M1>=0, M1*C == G];
obj = 0;
for k = 1:qx
    for j = 1:qs
        obj = obj + M1(k,j)^2;
    end
end
optimize(cons,obj)
M1 = value(M1);

T1 = sdpvar(qu,qs,'full');
cons = [T1*ones(qs,1) == eta1, T1>=0, T1*C == H*K1];
obj = 0;
for k = 1:qu
    for j = 1:qs
        obj = obj + T1(k,j)^2;
    end
end
optimize(cons,obj)
T1 = value(T1);


%% Pz and Pa
Pz = sdpvar(nx); 
Pa = sdpvar(qs);

cons = [Ak'*Pz*Ak - Pz + Qz - K1'*Qv*K1 <= -tol*eye(nx)];
cons = [cons, L1'*Pa*L1 - Pa + Qa <= -tol*eye(qs)];
obj = 0;
optimize(cons,obj);
Pz = value(Pz);
Pa = value(Pa);

toc

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
obj = obj + z(:,N)'*Pz*z(:,N) + (a(:,N)-1)'*Pa*(a(:,N)-1);


%% Optimize
optimize(cons,obj);
z = value(z);
v = value(v);
a = value(a);

toc

%% Optimal Control Simulation
disp('-------------------------------------------------')
disp('Simulating perturbed runs ...')
tic
x = zeros(nx,N);
x(:,1) = x0;
xs = zeros(nx,N,runs);

u = zeros(nu,N);
us = zeros(nu,N,runs);

w = zeros(nw,N);

for i = 1:runs

    for k = 1:N-1
        u(:,k) = v(:,k) + K1*(x(:,k)-z(:,k));
%         u(:,k) = v(:,k);
        w(:,k) = (w_max-w_min).*rand(nw,1) + w_min;
        x(:,k+1) = A*x(:,k) + B*u(:,k) + w(:,k);
        
    end

    xs(:,:,i) = x;
    us(:,:,i) = u;

end
toc



