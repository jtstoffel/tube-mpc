%% Homothetic Tube Optimal Control Example
% see pg. 7 for example 
% Josh Stoffel
disp('-------------------------------------------------')
disp('Starting Homothetic Tube Optimal Control Problem ...')

clear all; clc
opts = sdpsettings('solver','mosek','verbose',0);
tol = 1e-8;

%% Given in example
% State Space System
system = htmpc_example_model;
A = system.A;
B = system.B;
N = 20;
nx = length(A);
nu = size(B,2);

% Variable Bounds
x1_max = 10;
x1_min = -10;
x2_max = 2;
x2_min = -10;
u_max = 1;
u_min = -1;
X = Polyhedron([x1_min x2_min; x1_min x2_max; x1_max x2_min; x1_max x2_max]);
W = Polyhedron(0.1*[-1 -1; -1 1; 1 -1; 1 1]);
U = Polyhedron([u_min; u_max]);
% Xobs = Polyhedron([-4 -4; -2 -4; -4 2; -2 2]);
% X = X - Xobs;

% Initial and Final States
x0 = [-7; -2];
Xf = Polyhedron([6 -4; 4 0; 0 -1; -1 3]); % rough estimate

Qz = eye(2);
Qv = 0.01;

K = dlqr(A,B,Qz,Qv);

lambda = 0.6088;
mu = 0.9551;
alpha_bar = 2.4415;

qa = 0.01;
pa = 0.0158;

Ak = A-B*K;
S = compute_mrpi_set(Ak, W, 0.001);
S_nvert = length(S.V);
R = -K*S;
R_nvert = length(R.V);
S_plus = (A-B*K)*S;  
Pz = eye(2);

%% Decision Variables
v = sdpvar(nu,N);
% assign(v,zeros(nu,N))

z = sdpvar(nx,N);
% assign(z,zeros(nx,N))

a = sdpvar( 1,N);
% assign(a,ones(1,N))

%% Objective and Constraints (5.10a, 5.10b, 5.10c, 2.7, 2.8a, 2.8b)
objective = 0;
constraints = [];

% S_verts_new_initial = S.V * a(1) + repmat(z(:,1)', [S_nvert, 1]); % scaled and shifted mRPI

constraints = [constraints, z(:,1) == x0]; % initial state constraint

for k = 1:N-1
    objective = objective + ...
        z(:,k)'*Qz*z(:,k) + qa*(a(k)-alpha_bar)^2 + v(:,k)'*Qv*v(:,k); % quadratic cost function 
    
    constraints = [constraints, a(k) >= 0 ]; % positive scaling
    
    constraints = [constraints, z(:,k+1) == A*z(:,k) + B*v(:,k)]; % center line dynamics
    
    constraints = [constraints, a(k+1) == lambda*a(k) + mu]; % alpha next
    
    S_verts_new = S.V * a(k) + repmat(z(:,k)', [S_nvert, 1]); % scaled and shifted mRPI
    
    constraints = [constraints, x1_min <= S_verts_new(:,1) <= x1_max];
    constraints = [constraints, x2_min <= S_verts_new(:,2) <= x2_max];

%     
%     for n = 1:S_nvert
%         constraints = [constraints, ismember(S_verts_new(n,:)', X)]; % scaled and shifted mRPI is in X
%     end
    
    R_verts_new = R.V * a(k) + repmat(v(:,k)', [R_nvert, 1]); % scaled and shifted mRPI
    constraints = [constraints, u_min <= R_verts_new(:,1) <= u_max];
%     
% 
%     for n = 1:R_nvert
%         constraints = [constraints, ismember(R_verts_new(n,:)', U)]; % scaled and shifted mRPI is in U
%     end
end

objective  = objective + z(:,N)'*Pz*z(:,N) + pa*(a(N)-alpha_bar)^2; % final state cost

S_verts_new_final = S.V * a(N) + repmat(z(:,N)', [S_nvert, 1]); % scaled and shifted mRPI
    
for n = 1:S_nvert
    constraints = [constraints, ismember(S_verts_new_final(n,:)', Xf)]; % scaled and shifted mRPI is in X
end



%% Optimize
optimize(constraints,objective,opts);

%% Plotting
figure(1)
plot(X)
hold on
Xf.plot('color','lightblue')
axis equal
for i = 1:N
    S_verts_new = S.V * value(a(i)) + repmat(value(z(:,i))', [S_nvert, 1]);
    S_new = Polyhedron(S_verts_new);
    S_new.plot('color','green')
end
plot(value(z(1,:)),value(z(2,:)),'Linewidth', 2, 'color', 'white')
xlabel('x1')
ylabel('x2')


figure(2)
plot(value(v))
grid on 
xlabel('Time Step, k')
ylabel('Nominal Control Input, v(k)')





