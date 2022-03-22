function sys = planar_double_integrator_model()

%% Discrete State Space

% states: x1 v1 x2 v2
% controls: u1 u2

A = [1 1 0 0;
     0 1 0 0;
     0 0 1 1;
     0 0 0 1];

B = [0.5   0;
       1   0;
       0 0.5;
       0   1];

%% Initial State
x0 = [-5;
      -2;
       3;
       4;];

%% Bounds
x_min = [-10; -20; -10; -20];
x_max = -x_min;

u_min = [-1; -1];
u_max = -u_min;

w_min = [-0.1; -0.1; -0.1; -0.1];
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


