function plot_4D_etoc(system, tube, simdata)
%% System Data
X = system.X;
C = system.C;

%% Tube Data
N = tube.N;
z = tube.z;
z1 = z(1,:);
z2 = z(2,:);
z3 = z(3,:);
z4 = z(4,:);
v1 = tube.v(1,:);
v2 = tube.v(2,:);
a = tube.a;

%% Simulation Data
runs = simdata.runs;
xs = simdata.xs;
u1 = simdata.us(1,:,1);
u2 = simdata.us(2,:,1);

%% State Trajectory Plot
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

title(upper(strrep(system.name, '_',' ')))

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

%% Control Input vs. Time Plot
figure
subplot(2,1,1)
grid on; hold on 
stairs(v1)
stairs(u1)
stairs(u1 - v1)
plot(xlim, ones(2,1) * system.u_min(1), 'k--')
plot(xlim, ones(2,1) * system.u_max(1), 'k--')
xlabel('Time Step, k')
ylabel('Control Input 1')
legend('Nominal', 'Total', 'Disturbance Rejection', 'Input Constraint')

subplot(2,1,2)
grid on; hold on
stairs(v2)
stairs(u2)
stairs(u2 - v2)
plot(xlim, ones(2,1) * system.u_min(2), 'k--')
plot(xlim, ones(2,1) * system.u_max(2), 'k--')
xlabel('Time Step, k')
ylabel('Control Input 2')
legend('Nominal', 'Total', 'Disturbance Rejection', 'Input Constraint')

%% State vs. Time Plot
figure
subplot(2,2,1)
grid on; hold on
stairs(z1)
plot(xlim, ones(2,1) * system.x_min(1), 'k--')
plot(xlim, ones(2,1) * system.x_max(1), 'k--')
xlabel('Time Step, k')
ylabel('Nominal X Position')

subplot(2,2,2)
grid on; hold on
stairs(z2)
xlabel('Time Step, k')
ylabel('Nominal X Velocity')
plot(xlim, ones(2,1) * system.x_min(2), 'k--')
plot(xlim, ones(2,1) * system.x_max(2), 'k--')

subplot(2,2,3)
grid on; hold on
stairs(z3)
plot(xlim, ones(2,1) * system.x_min(3), 'k--')
plot(xlim, ones(2,1) * system.x_max(3), 'k--')
xlabel('Time Step, k')
ylabel('Nominal Y Position')

subplot(2,2,4)
grid on; hold on
stairs(z4)
plot(xlim, ones(2,1) * system.x_min(4), 'k--')
plot(xlim, ones(2,1) * system.x_max(4), 'k--')
xlabel('Time Step, k')
ylabel('Nominal Y Velocity')








