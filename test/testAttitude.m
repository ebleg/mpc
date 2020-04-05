clear; clear attitudeMPC; clear positionMPC;

addpath('..');
addpath('../fun');
addpath('../fun/ctrl');
addpath('../fun/mod');
addpath('../fun/vis');
addpath('../tools');
run parameters

% xref=zeros(par.angCtrl.dim.x, par.angCtrl.dim.N+1);
% for i=1:par.angCtrl.dim.N+1
%     xref(:,i) = [1 1 1 phi(i) theta(i) psi(i)]';
% end
% 
% att = xref(:,1);
% 
% u_input = [1 1 1]';
% uref=zeros(par.angCtrl.dim.u, par.angCtrl.dim.N);
% for i=1:par.angCtrl.dim.N
%     uref(:,i) = u_input;
% end
% 
% t = 0:pi/100:pi;
% phi = 5*cos(t);
% theta = 5*sin(t);
% psi = t;
% plot3(phi,theta,psi,'*r');
% for i=1:length(phi)
%   plot3(phi(i),theta(i),psi(i),'*r');
%   hold on;
%   pause(0.01);
% end

par.sim.tmax = 20;

% path = @(t) [0*t; 0*t; t]; % Fly straight up
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
xref = [sol.u.pos(2:3,:); ref.x.ang(3:6,:)];

predictionBuffer = ceil(par.angCtrl.dim.N*par.angCtrl.predInt/par.sim.h);

% i=2:(nsteps-predictionBuffer)
for i=2:250
    disp(num2str(i));
    sol.u.ang(:,i) = attitudeMPC(xref(:,i:end), par, sol.t(i), sol.x.ang(:,i-1), [],[]);
    g = @(x) rotationalDynamics(x, [sol.u.pos(1,i); sol.u.ang(:,i)] , par);
    sol.x.ang(:,i) = GL4(g, sol.x.ang(:,i-1), par);
end

close all;
figure; ax = gca; axis equal; grid; grid minor; hold on;
title('Quadcopter simulation'); xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
% refPlot = plotTrajectory(ax, ref.t, ref.x.pos, '.', 'Reference trajectory');
% refPlot = plotTrajectory(ax, ref.t, ref.x.pos, '.', 'Reference trajectory');
solPlot = plotTrajectory(ax, sol.t, sol.x.pos, '.', 'Simulated trajectory');
legend();
% simulateDrone(ax, sol, par);
