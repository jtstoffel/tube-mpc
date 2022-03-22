function postprocess_planar_double_integrator(system, tube, simdata)
%% System Data
X = system.X;
C = system.C;

%% Tube Data
N = tube.N;
z = tube.z;
v1 = tube.v(1,:);
v2 = tube.v(2,:);
a = tube.a;

%% Simulation Data
runs = simdata.runs;
xs = simdata.xs;
u1 = simdata.us(1,:,1);
u2 = simdata.us(2,:,1);

%% Plots
figure
hold on
axis equal
X_pos = X.projection([1 3]);
X_pos.plot('color', [0 0 0])

for k = 1:N
    Sa = Polyhedron('A', C, 'b', a(:,k));
    Sa = Sa.plus(z(:,k));
    Sa = Sa.projection([1 3]);
    Sa.minHRep;
    Sa.plot('color', 'green')
end
plot(z(1,:),z(3,:),'bo',LineWidth=3)

for i = 1:runs
    plot(xs(1,:,i),xs(3,:,i),'c',LineWidth=0.05)
    plot(xs(1,:,i),xs(3,:,i),'co',LineWidth=0.2)
end

title('Planar Double Integrator Position')

figure
hold on
axis equal
X_vel = X.projection([2 4]);
X_vel.plot('color', [0 0 0])

for k = 1:N
    Sa = Polyhedron('A', C, 'b', a(:,k));
    Sa = Sa.plus(z(:,k));
    Sa = Sa.projection([2 4]);
    Sa.plot('color', 'green')
end
plot(z(2,:),z(4,:),'bo',LineWidth=3)

for i = 1:runs
    plot(xs(2,:,i),xs(4,:,i),'c',LineWidth=0.05)
    plot(xs(2,:,i),xs(4,:,i),'co',LineWidth=0.2)
end

plot(0,0,'ro', LineWidth=2)
title('Planar Double Integrator Velocity')

xlabel('v1')
ylabel('v2')


figure
grid on; hold on 
plot(v1)
plot(v2)
plot(u1)
plot(u2)
plot(u1 + u2 - v1 - v2)
plot(xlim, ones(2,1) * system.u_min(1), 'k--')
plot(xlim, ones(2,1) * system.u_max(1), 'k--')
xlabel('Time Step, k')
ylabel('Control Input')
legend('Nominal Input 1', 'Nominal Input 2', 'Total Input 1', 'Total Input 2','Disturbance Rejection', 'Input Constraint')