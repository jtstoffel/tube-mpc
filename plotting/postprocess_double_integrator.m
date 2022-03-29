function postprocess_double_integrator(system, tube, simdata)
%% System Data
X = system.X;
C = system.C;

%% Tube Data
N = tube.N;
z = tube.z;
z1 = z(1,:);
z2 = z(2,:);
v = tube.v;
a = tube.a;

%% Simulation Data
runs = simdata.runs;
xs = simdata.xs;
u = simdata.us(:,:,1);

%% State Trajectory Plot
figure
hold on
axis equal
X.plot('color', [0 0 0])

for k = 1:N
    Sa = Polyhedron('A', C, 'b', a(:,k));
    Sa = Sa.plus(z(:,k));
    Sa.plot('color','green');
end
plot(z(1,:),z(2,:),'bo',LineWidth=3)

for i = 1:runs
    plot(xs(1,:,i),xs(2,:,i),'c',LineWidth=0.05)
    plot(xs(1,:,i),xs(2,:,i),'co',LineWidth=0.2)
end

plot(0,0,'ro', LineWidth=2)
title('Double Integrator Trajectory')

xlabel('x1')
ylabel('x2')

%% Control Input vs. Time Plot
figure
grid on; hold on 
plot(v)
plot(u)
plot(u-v)
plot(xlim, ones(2,1) * system.u_min, 'k--')
plot(xlim, ones(2,1) * system.u_max, 'k--')
xlabel('Time Step, k')
ylabel('Control Input'); 
legend('Nominal', 'Total','Disturbance Rejection', 'Input Constraint')

%% State vs. Time Plot
figure
subplot(2,1,1)
grid on; hold on
plot(z1)
plot(xlim, ones(2,1) * system.x_min(1), 'k--')
plot(xlim, ones(2,1) * system.x_max(1), 'k--')
xlabel('Time Step, k')
ylabel('Nominal Position')

subplot(2,1,2)
grid on; hold on
plot(z2)
xlabel('Time Step, k')
ylabel('Nominal Velocity')
plot(xlim, ones(2,1) * system.x_min(2), 'k--')
plot(xlim, ones(2,1) * system.x_max(2), 'k--')