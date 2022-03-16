function sys = spring_mass_damper_model()

%% State-Space
A = [0 1; -1 -1];

B = [0; 1];

C = eye(2);
D = zeros(size(B));

sysc = ss(A,B,C,D);


%% Discretization 
Ts = 1;
sysd = c2d(sysc,Ts);
A = sysd.A;
B = sysd.B;

%% Initial State
% x0 = [2 -1]';
x0 = [4 3]';

%% Bounds
x_min = [-5 -10]';
x_max = -x_min;

u_min = -0.5;
u_max = -u_min;

w_min = [-0.0001 -0.1]';
w_max = -w_min;

%% System
sys.A = A;
sys.B = B;
sys.x0 = x0;
sys.x_min = x_min;
sys.x_max = x_max;
sys.u_min = u_min;
sys.u_max = u_max;
sys.w_min = w_min;
sys.w_max = w_max;
sys.name = 'spring_mass_damper_model';
sys.nx = 2;
sys.nu = 1;
sys.nw = 2;
