clear; clear attitudeMPC; clear positionMPC;

addpath('..');
addpath('../fun');
addpath('../fun/ctrl');
addpath('../fun/mod');
addpath('../fun/vis');
addpath('../tools');
run parameters
run initLTI

path = @(t) [4*cos(t); 4*sin(t); t/3]; %Ellipsoidal spiral

sol = struct();
sol.t = (0:par.sim.h:par.sim.tmax);
nsteps = numel(sol.t);

sol.x.pos = nan(par.posCtrl.dim.x, nsteps);
sol.x.ang = nan(par.angCtrl.dim.x, nsteps);
sol.u.pos = nan(par.posCtrl.dim.u, nsteps);
sol.u.ang = nan(par.angCtrl.dim.u, nsteps);

ref = generateReference(sol.t, path, par);

sol.x.ang(:,1) = ref.x.ang(:,1);
% sol.x.ang = ref.x.ang;
% sol.u.ang = ref.u.ang;
sol.u.pos = ref.u.pos;
sol.x.pos = ref.x.pos;
% xref = ref.x.ang;
yref(:,1) = LTI.C*ref.x.ang(:,1);

x_1 = LTI.x0;
xehat_1=[ref.x.ang(:,1); LTI.d];

predictionBuffer = ceil(par.angCtrl.dim.N*par.angCtrl.predInt/par.sim.h);

% i=2:(nsteps-predictionBuffer)
for i=2:50
    disp(num2str(i));
    [u, x_0, xehat_0, e] = attitudeMPC(LTI, LTI_e, par, yref(:,i-1), pred, x_1, xehat_1, sol.t(i));
    sol.u.ang(:,i) = u;
    error(:,i) = e;
    x_1 = x_0;
    xehat_1 = xehat_0;
    % Regular MPC
    % sol.u.ang(:,i) = attitudeMPC(ref.x.ang(:,i), par, sol.t(i), sol.x.ang(:,i-1));
    g = @(x) rotationalDynamics(x, [sol.u.pos(1,i); sol.u.ang(:,i)] , par);
    sol.x.ang(:,i) = RK4(g, sol.x.ang(:,i-1), par.sim.h);
%     sol.x.ang(:,i) = ref.x.ang(:,i);
    yref(:,i) = LTI.C * sol.x.ang(:,i-1);
end

% close all;
% figure();
% plot(error(1,:),'b')
% hold on
% plot(error(2,:),'r')
% plot(error(3,:),'y')
% plot(error(4,:),'g')
% plot(error(5,:),'m')
% plot(error(6,:),'c')
% title('Offset free output MPC'); ylabel('Error'); xlabel('Iteration');

close all;
figure; ax = gca; axis equal; grid; grid minor; hold on;
title('Quadcopter simulation'); xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
% refPlot = plotTrajectory(ax, ref.t, ref.x.pos, '.', 'Reference trajectory');
solPlot = plotTrajectory(ax, sol.t, sol.x.pos, '.', 'Simulated trajectory');
legend();
% simulateDrone(ax, sol, par);
