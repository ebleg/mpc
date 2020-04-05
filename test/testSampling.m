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

frame = par.posCtrl.predInt/par.angCtrl.predInt;
sol = struct();
sol.t = (0:par.sim.h:par.sim.tmax);
nsteps = numel(sol.t);

sol.x.pos = nan(par.posCtrl.dim.x, nsteps);
sol.x.ang = nan(par.angCtrl.dim.x, frame*nsteps);
sol.u.pos = nan(par.posCtrl.dim.u, nsteps);
sol.u.ang = nan(par.angCtrl.dim.u, frame*nsteps);

ref = generateReference(sol.t, path, par);

sol.x.ang(:,1:frame) = repelem(ref.x.ang(:,1),1,frame);
% sol.x.ang = ref.x.ang;
% sol.u.ang = ref.u.ang;
sol.x.pos = ref.x.pos;
% sol.u.pos = ref_pos.u.pos;
xref = repelem(ref.x.ang,1,frame);

% xref = [sol.u.pos(2:3,:); ref.x.ang(3:6,:)];
predictionBuffer = ceil(par.angCtrl.dim.N*par.angCtrl.predInt/par.sim.h);

% i=2:(nsteps_pos-predictionBuffer)
for i=2:5
    sol.u.pos(:,i) = ref.u.pos(:,i);
%     sol.u.pos(:,i) = positionMPC(sol.x.ang(:,frame*(i-2)+10), ...
%                                  sol.x.pos(:,i-1), ...
%                                  sol.t.pos(i), ...
%                                  ref_pos, par);
    temp_u = nan(par.angCtrl.dim.u,frame+1);
    temp_x = nan(par.angCtrl.dim.x,frame+1);
    temp_x(:,1) = sol.x.ang(:,frame*(i-1));
    for j=2:frame+1
        disp([num2str(i), ', ' num2str(j)]);
        % sol.t.ang(frame*(i-2)+j)
        temp_u(:,j) = attitudeMPC(xref(:,frame*(i-2)+j:end), par, 1, temp_x(:,j-1));
        g = @(x) rotationalDynamics(x, [sol.u.pos(1,i); temp_u(:,j)] , par);
        temp_x(:,j) = GL4(g, temp_x(:,j-1), par);
    end
    sol.u.ang(:,frame*(i-2)+2:frame*(i-2)+11) = temp_u(:,2:frame+1);
    sol.x.ang(:,frame*(i-1)+1:frame*(i-1)+10) = temp_x(:,2:frame+1);    
end
sol.x.pos = repelem(sol.x.pos,1,frame);
sol.u.pos = repelem(sol.u.pos,1,frame);
sol.t = repelem(sol.t,1,frame);

close all;
figure; ax = gca; axis equal; grid; grid minor; hold on;
title('Quadcopter simulation'); xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
% refPlot = plotTrajectory(ax, ref.t, ref.x.pos, '.', 'Reference trajectory');
% refPlot = plotTrajectory(ax, ref.t, ref.x.pos, '.', 'Reference trajectory');
solPlot = plotTrajectory(ax, sol.t, sol.x.pos, '.', 'Simulated trajectory');
legend();
% simulateDrone(ax, sol, par);
