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

path = @(t) [0*t; 0*t; t]; % Fly straight up
% path = @(t) [4*cos(t); 4*sin(t); t/3]; %Ellipsoidal spiral

sol = struct();
sol.t.pos = (0:par.sim.h:par.sim.tmax);
sol.t.ang = (0:par.sim.h*(par.angCtrl.sampleInt/par.posCtrl.sampleInt):par.sim.tmax);
nsteps_pos = numel(sol.t.pos);
nsteps_ang = numel(sol.t.ang);

sol.x.pos = nan(par.posCtrl.dim.x, nsteps_pos);
sol.x.ang = nan(par.angCtrl.dim.x, nsteps_ang);
sol.u.pos = nan(par.posCtrl.dim.u, nsteps_pos);
sol.u.ang = nan(par.angCtrl.dim.u, nsteps_ang);

ref_pos = generateReference(sol.t.pos, path, par);
ref_ang = generateReference(sol.t.ang, path, par);

frame = par.posCtrl.sampleInt/par.angCtrl.sampleInt;

sol.x.ang(:,1:frame) = ref_ang.x.ang(:,1:frame);
% sol.x.ang = ref.x.ang;
% sol.u.pos = ref_pos.u.pos;
sol.x.pos = ref_pos.x.pos;
ang = ref_ang.x.ang(:,1);

predictionBuffer = ceil(par.angCtrl.dim.N*par.angCtrl.predInt/par.sim.h);

% i=2:(nsteps_pos-predictionBuffer)
for i=2:(nsteps_pos -predictionBuffer)
    sol.u.pos(:,i) = positionMPC(sol.x.ang(:,frame*(i-2)+10), ...
                                 sol.x.pos(:,i-1), ...
                                 sol.t.pos(i), ...
                                 ref_pos, par);
    temp_u = nan(par.angCtrl.dim.u,frame+1);
    temp_x = nan(par.angCtrl.dim.x,frame+1);
    temp_x(:,1) = sol.x.ang(:,frame*(i-1));
    for j=2:frame+1
        disp([num2str(i), ', ' num2str(j)]);
        temp_u(:,j) = attitudeMPC(ref_ang, par, sol.t.ang(frame*(j-1)+j), temp_x(:,j-1), [],[]);
        omega = -temp_u(3,j)/par.drone.rotor.Km;
    %     omega = [-1 1 -1 1]*par.drone.u2omega*[sol.u.pos(1,i) sol.u.ang(:,i)']';
    %     sol.x.ang(:,i) = [zeros(3,1); sol.u.pos(2:3,i); ref.x.ang(6,i)];
        temp_x(:,j) = rotationalDynamics([zeros(2,1); temp_x(3:6,j-1)], [temp_u(:,j); omega] , par);
    %     sol.x.ang(:,i) = rotationalDynamics(sol.x.ang(:,1), [sol.u.ang(:,i); omega] , par);
    end
    sol.u.ang(:,frame*(i-2)+2:frame*(i-2)+11) = temp_u(:,2:frame+1);
    sol.x.ang(:,frame*(i-1)+1:frame*(i-1)+10) = temp_x(:,2:frame+1);    
end

close all;
figure; ax = gca; axis equal; grid; grid minor; hold on;
title('Quadcopter simulation'); xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
% refPlot = plotTrajectory(ax, ref.t, ref.x.pos, '.', 'Reference trajectory');
% refPlot = plotTrajectory(ax, ref.t, ref.x.pos, '.', 'Reference trajectory');
solPlot = plotTrajectory(ax, sol.t.pos, sol.x.pos, '.', 'Simulated trajectory');
legend();
simulateDrone(ax, sol, par);
