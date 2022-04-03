function plot_rrtstar_etoc(system, map, global_trajectory, z0, zf)

%% Trajectory Plot
figure(1)
cmap = cool;
X_pos = system.X.projection([1 3]);
X_pos.plot('wire',true)
hold on; axis equal
map.plot('color', 'blue')
for i = 1:length(length(global_trajectory))
    color = cmap(randi(256),:);
    z = global_trajectory(i).z;
    v = global_trajectory(i).v;
    a = global_trajectory(i).a;
    for k = 1:global_trajectory(i).N
        Sa = Polyhedron('A', system.C, 'b', a(:,k));
        Sa = Sa.plus(z(:,k));
        Sa = Sa.projection([1 3]);
        Sa.minHRep;
        Sa.plot('color', color)
    end
    plot(z(1,:),z(3,:),'m',LineWidth=3)
end
scatter(z0(1), z0(3), 'ro',LineWidth=3)
scatter(zf(1), zf(3), 'go',LineWidth=3)
set(gca,'FontSize',18)


%% State and Control Plots
zs = [];
vs = [];
as = [];
costs = [];
for i = 1:length(global_trajectory)
    zs = [zs, global_trajectory(i).z];
    vs = [vs, global_trajectory(i).v];
    as = [as, global_trajectory(i).a];
    costs = [costs, global_trajectory(i).cost];
end

figure(2)
sgtitle('Nominal State Trajectories','FontSize',18)
subplot(2,2,1)
stairs(zs(1,:),LineWidth=2)
ylabel('z1')
xlabel('Time step')
set(gca,'FontSize',18)

subplot(2,2,2)
stairs(zs(2,:),LineWidth=2)
ylabel('z2')
xlabel('Time step')
set(gca,'FontSize',18)

subplot(2,2,3)
stairs(zs(3,:),LineWidth=2)
ylabel('z3')
xlabel('Time step')
set(gca,'FontSize',18)

subplot(2,2,4)
stairs(zs(4,:),LineWidth=2)
ylabel('z4')
xlabel('Time step')
set(gca,'FontSize',18)

figure(3)
vs(isnan(vs)) = 0;
sgtitle('Nominal Control Input','FontSize',18)
subplot(2,1,1)
stairs(vs(1,:),LineWidth=2)
ylabel('v1')
xlabel('Time step')
set(gca,'FontSize',18)

subplot(2,1,2)
stairs(vs(2,:),LineWidth=2)
ylabel('v2')
xlabel('Time step')
set(gca,'FontSize',18)

%% Cost Plot
figure(6)
stairs(costs,LineWidth=2)
ylabel('Cost')
xlabel('Trajectory Tube Section')
set(gca,'xtick',1:length(global_trajectory))
set(gca,'FontSize',18)

end

