function sysout = preprocess_system(system, useDataFile)
% Preprocess offline tube MPC parameters

%% Check Preprocess Data
fn = ['./data/', system.name, '_data.mat'];
if useDataFile
    load(fn, 'system');
else
    disp('-------------------------------------------------')
    disp('Calculating Tube Approximation Parameters ...')

    %% Solver Settings
    opts = sdpsettings('solver','mosek','verbose',0);
    tol = 1e-8;

    %% State Space System
    A = system.A;
    B = system.B;

    %% State Constraints
    G = normalized_inequality_constraint(system.x_min, system.x_max); system.G = G;
    qx = length(G); system.qx = qx;
    X = Polyhedron('A',G,'b',ones(qx,1)); system.X = X;

    %% Input Constraints
    H = normalized_inequality_constraint(system.u_min, system.u_max); system.H = H;
    qu = length(H); system.qu = qu;
    U = Polyhedron('A',H,'b',ones(qu,1)); system.U = U;

    %% Disturbance Constraints
    D = normalized_inequality_constraint(system.w_min, system.w_max); system.D = D;
    qw = length(D); system.qw = qw;
    W = Polyhedron('A',D,'b',ones(qw,1)); system.W = W;

    %% State and Control Weighting
    Qz = eye(system.nx); system.Qz = Qz;
    Qv = 0.1*eye(system.nu); system.Qv = Qv;

    %% S(1), K(1), and C
    K1 = -dlqr(A,B,Qz,Qv); system.K1 = K1;
    Ak = A+B*K1;  system.Ak = Ak;
    S1 = compute_mrpi_set(Ak,W, 1e-5);
    S1.computeHRep; system.S1 = S1;

    C = zeros(size(S1.A));
    qs = length(C); system.qs = qs;
    for k = 1:length(S1.A)
        if S1.b(k) < 0
            C(k,:) = S1.A(k,:) ./ -S1.b(k)';
        else
            C(k,:) = S1.A(k,:) ./  S1.b(k)';
        end
    end
    system.C = C;

    %% Tube Elasticity Weighting
    Qa = 10 * eye(qs); system.Qa = Qa;

    %% d, gamma1, eta1
    d = W.support(C'); system.d = d; % Eq 3.4
    gamma1 = S1.support(G'); system.gamma1 = gamma1; % Eq 3.4
    K1S1 = K1*S1; system.K1S1 = K1S1;
    eta1 = K1S1.support(H'); system.eta1 = eta1;

    %% L(1), M(1), T(1)
    L1 = sdpvar(qs,qs,'full');
    cons = [L1*ones(qs,1) == 1-d, L1>=0, L1*C == C*Ak];
    obj = 0;
    for k = 1:qs
        for j = 1:qs
            obj = obj + L1(k,j)^2;
        end
    end
    optimize(cons,obj,opts)
    L1 = value(L1); system.L1 = L1;

    M1 = sdpvar(qx,qs,'full');
    cons = [M1*ones(qs,1) == gamma1, M1>=0, M1*C == G];
    obj = 0;
    for k = 1:qx
        for j = 1:qs
            obj = obj + M1(k,j)^2;
        end
    end
    optimize(cons,obj,opts)
    M1 = value(M1); system.M1 = M1;

    T1 = sdpvar(qu,qs,'full');
    cons = [T1*ones(qs,1) == eta1, T1>=0, T1*C == H*K1];
    obj = 0;
    for k = 1:qu
        for j = 1:qs
            obj = obj + T1(k,j)^2;
        end
    end
    optimize(cons,obj,opts)
    T1 = value(T1); system.T1 = T1;

    %% Pz and Pa
    Pz = sdpvar(system.nx);
    Pa = sdpvar(qs);

    cons = [Ak'*Pz*Ak - Pz + Qz - K1'*Qv*K1 <= -tol*eye(system.nx)];
    cons = [cons, L1'*Pa*L1 - Pa + Qa <= -tol*eye(qs)];
    obj = 0;
    optimize(cons,obj,opts);
    Pz = value(Pz); system.Pz = Pz;
    Pa = value(Pa); system.Pa = Pa;

    %% Save Preprocessed Model Data
    save(fn, 'system')
end

%% Output Preprocessed System
sysout = system;
sysout.preprocessed = true;

end

