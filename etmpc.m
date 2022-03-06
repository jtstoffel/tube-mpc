%% Elastic Tube MPC Example
% Josh Stoffel
clear all; clc
opts = sdpsettings('solver','mosek','verbose',0);
tol = 1e-8;


sys1 = struct(...
    'A', [1 1; 0 1], ...
    'B', [0.5;1], ...
    'x0', [-6.5; -2], ...
    'x_min', [-10 -10]', ...
    'x_max', [10 2]', ...
    'u_min', -1, ...
    'u_max', 1, ...
    'w_min', [-0.2 -0.2]', ...
    'w_max', [0.2 0.2]');

sys2 = struct(...
    'A', [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0], ...
    'B', [0 0; 0 0; 1 0; 0 1], ...
    'x0', [1 1 0 0]', ...
    'x_min', [-10 -10 -20 -20]', ...
    'x_max', [10 10 20 20]', ...
    'u_min', [-1 -1]', ...
    'u_max', [1 1]', ...
    'w_min', [-0.1 -0.1 -0.1 -0.1]', ...
    'w_max', [0.1 0.1 0.1 0.1]');


system = sys1;
only_nominal_control = false;
no_disturbance = false;
max_disturbance = false;
min_disturbance = false;

%% State Space System
A = system.A;
B = system.B;
x0 = system.x0;
N = 20;
runs = 5;
[nx, nu] = size(B);
nw = nx;


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
D = normalized_inequality_constraint(w_min, w_max);
qw = length(D);
W = Polyhedron('A',D,'b',ones(qw,1));


%% Weighting Matricies
Qz = eye(nx);
Qv = 0.1*eye(nu);



%% S(1), K(1), and C
K1 = -dlqr(A,B,Qz,Qv);
Ak = A+B*K1;
S1 = compute_mrpi_set(Ak,W, 0.001);
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

Qa = 0.05*eye(qs);



%% d, gamma1, eta1
d = zeros(qs,1);
for k = 1:qs
    d(k,1) = W.support(C(k,:)');
end

gamma1 = zeros(qx,1);
for k = 1:qx
    gamma1(k,1) = S1.support(G(k,:)');
end

eta1 = zeros(qu,1);
K1S1 = K1*S1;
for k = 1:qu
    eta1(k,1) = K1S1.support(H(k,:)');
end



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
optimize(cons,obj)
Pz = value(Pz);
Pa = value(Pa);





%% MPC Loop
T = 15; % final time = mpc horizon
x = zeros(nx,T);
x(:,1) = x0;
xs = zeros(nx,T,runs);

u = zeros(nu,T);
us = zeros(nu,T,runs);

zs = zeros(nx,N,T,runs);
vs = zeros(nu,N,T,runs);
as = zeros(qs,N,T,runs);


for i = 1:runs

    for j = 1:T-1 
        % Decision Variables
        v = sdpvar(nu,N);
        z = sdpvar(nx,N);
        a = sdpvar(qs,N);

        % Objective and Constraints
        obj = 0;
        cons = [];
        cons = [cons, -a <= 0]; % positive vector
        cons = [cons, C*x(:,j) - C*z(:,1) - a(:,1) <= 0]; % initial state set constraint

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

        % Optimize
        optimize(cons,obj);
        z = value(z);
        v = value(v);
        a = value(a);

        % Calculate Control
        if only_nominal_control
            u(:,k) = v(:,k);
        else
            u(:,j) = v(:,1) + K1*(x(:,j)-z(:,1));
        end

        % Calculate Random Disturbance
        if no_disturbance
            w = zeros(nw,1);
        elseif max_disturbance
            w = w_max;
        elseif min_disturbance
            w = w_min;
        else
            w = (w_max-w_min).*rand(nw,1) + w_min;
        end

        % Update State
        x(:,j+1) = A*x(:,j) + B*u(:,j) + w;
        
        % Save Tube Data
        zs(:,:,j,i) = z;
        vs(:,:,j,i) = v;
        as(:,:,j,i) = a;

    end

    % Save State and Control
    xs(:,:,i) = x;
    us(:,:,i) = u;

end


%% Plotting

figure(3)
hold on
axis equal
X.plot('color', [0 0 0]) % plot state domain


for j = 1:T

    for k = 1:N
        Sa = Polyhedron('A', C, 'b', as(:,k,j,1));
        Sa.computeVRep;
        state_set_verts = repmat(zs(:,k,j,1)',[length(Sa.V), 1]) + Sa.V;
        shifted_state_set = Polyhedron(state_set_verts);
        shifted_state_set.plot('color', [0.06*j 0.7 0.05*N]);

    end
end

plot(xs(1,:,1),xs(2,:,1),'r',LineWidth=1)
plot(xs(1,:,1),xs(2,:,1),'ro',LineWidth=1)







% for i = 1:runs
%     plot(xs(1,:,i),xs(2,:,i),'c',LineWidth=0.05)
%     plot(xs(1,:,i),xs(2,:,i),'co',LineWidth=0.2)
% end
% xlabel('x1')
% ylabel('x2')




% 
% figure(4)
% grid on; hold on 
% plot(v)
% plot(u)
% plot(u-v)
% xlabel('Time Step, k')
% ylabel('Control Input')
% legend('Nominal', 'Total','Disturbance Rejection')

