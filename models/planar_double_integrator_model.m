function sys = planar_double_integrator_model()
%% State-Space
A = [0 0 1 0 ; 
     0 0 0 1 ; 
     0 0 0 0 ;
     0 0 0 0];

B = [0 0 ;
     0 0 ; 
     1 0 ; 
     0 1];

C = eye(4);
D = zeros(size(B));

sysc = ss(A,B,C,D);


%% Discretization 
Ts = 1;
sysd = c2d(sysc,Ts);
A = sysd.A;
B = sysd.B;

%% Initial State
x0 = [8 13 -1 4]';

%% Bounds
x_min = [-20 -20 -50 -50]';
x_max = -x_min;

u_min = [-2 -2]';
u_max = -u_min;

w_min = [-0.1 -0.1 -0.001 -0.001]';
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
sys.name = 'planar_double_integrator_model';
sys.nx = 4;
sys.nu = 2;
sys.nw = 4;
