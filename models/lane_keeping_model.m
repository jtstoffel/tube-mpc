function sys = lane_keeping_model()

%% Parameters
V0 = 8.33; % m/s
Cf = 133000; % N/rad
Cr = 98800; % N/rad
M = 1650; %kg
a = 1.11; % m
b = 1.59; % m
Iz = 2315.3; % m^2 kg
g = 9.81; % m/s^2
amax = 0.3 * g; % m/s^2
ts = 1; % s

%% State-Space
A = [0                      1   V0                                0; ...
     0        -(Cf+Cr)/(M*V0)    0             (b*Cr - a*Cf)/(M*V0); ...
     0                      0    0                                1; ...
     0  (b*Cr - a*Cf)/(Iz*V0)    0    ((a^2)*Cf + (b^2)*Cr)/(Iz*V0);];

A = eye(4) + A*ts; % discrete

B = [0; Cf/M; 0; a*Cf/Iz];

B = B*ts; % discrete

E = [0; 0; -1; 0];

%% Initial State
x0 = [0.5 0 0 0]';

%% Bounds
x_min = [-0.9 -20 -90*pi/180 -20*pi/180]';
x_max = -x_min;

u_min = -45*pi/180;
u_max = -u_min;

curvature_min = 0.5*V0^2/amax;
w_min = E * ts * curvature_min;
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
sys.name = 'Lane Keeping Model';

