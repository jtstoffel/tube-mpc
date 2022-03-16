function postprocess_htmpc_example(system, tube, simdata)
%% System Data
X = system.X;
C = system.C;

%% Tube Data
N = tube.N;
z = tube.z;
v = tube.v;
a = tube.a;

%% Simulation Data
runs = simdata.runs;
xs = simdata.xs;
u = simdata.us(:,:,1);

%% Plots
figure
hold on
axis equal
X.plot('color', [0 0 0])


for k = 1:N
    Sa = Polyhedron('A', C, 'b', a(:,k));
    Sa.computeVRep;
    state_set_verts = repmat(z(:,k)',[length(Sa.V), 1]) + Sa.V;
    shifted_state_set = Polyhedron(state_set_verts);
    shifted_state_set.plot('color','green');
end
plot(z(1,:),z(2,:),'bo',LineWidth=3)

for i = 1:runs
    plot(xs(1,:,i),xs(2,:,i),'c',LineWidth=0.05)
    plot(xs(1,:,i),xs(2,:,i),'co',LineWidth=0.2)
end

plot(0,0,'ro', LineWidth=2)


xlabel('x1')
ylabel('x2')

figure
grid on; hold on 
plot(v)
plot(u)
plot(u-v)
xlabel('Time Step, k')
ylabel('Control Input')
legend('Nominal', 'Total','Disturbance Rejection')