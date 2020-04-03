clear

addpath('..');
addpath('../fun');
addpath('../fun/ctrl');
addpath('../fun/mod');
addpath('../fun/vis');
addpath('../tools')
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
path = @(t) [4*cos(t); 4*sin(t); t/3]; %Ellipsoidal spiral

sol = struct();
sol.t = (0:par.sim.h:par.sim.tmax);
nsteps = numel(sol.t);

sol.x.pos = nan(par.posCtrl.dim.x, nsteps);
sol.x.ang = nan(par.angCtrl.dim.x, nsteps);
sol.u.pos = nan(par.posCtrl.dim.u, nsteps);
sol.u.ang = nan(par.angCtrl.dim.u, nsteps);
dx_ang = nan(par.angCtrl.dim.x, nsteps);

ref = generateReference(sol.t, path, par);

% sol.x.ang(:,1) = ref.x.ang(:,1);
sol.x.ang = ref.x.ang;
dx_ang(:,1) = [0 0 0 0 0 0];

predictionBuffer = ceil(par.angCtrl.dim.N*par.angCtrl.predInt/par.sim.h);

% i=2:(nsteps-predictionBuffer)
for i=2:20
    sol.x.ang(:,i) = [dx_ang(1:3,i-1); sol.u.pos(2:3,i); ref.x.ang(6,i)];
    sol.u.ang(:,i) = attitudeMPC(ref, par, sol.t, sol.x.ang(:,i-1), [],[]);
    omega = [-1 1 -1 1]*[ref.u.pos(1,i) sol.u.ang(:,i)']';
    dx_ang(:,i) = rotationalDynamics(sol.x.ang(:,i), [sol.u.ang(:,i); omega], par);
end

% close all;
% figure;  ax = gca; axis equal; grid; grid minor; hold on;
% refPlot = plotTrajectory(ax, ref.t, ref.x.pos, '.', 'Reference trajectory');
% solPlot = plotTrajectory(ax, sol.t, sol.x.pos, '.', 'Simulated trajectory');
% 
% for i=2:length(attitude)
%     p = struct();
%     dronePos = ref.x.pos(4:6,i);
%     rotorPts = [1 0 0; 0 1 0; -1 0 0; 0 -1 0]'.*par.drone.l; 
%     R = eul2rotm(attitude(:,i)', 'XYZ');
%     droneZaxis = R*[0 0 1]';
%     for l=1:4
%         rotorPts(:,l) = R*rotorPts(:,l) + dronePos;        
%     end
%     
%     Tvec = par.drone.u2omega*[ref.u.pos(1,i); sol.u.ang(:,i)];
%     Tvec = repmat(Tvec', 3, 1);
%     droneZaxis = repmat(droneZaxis, 1, 4);
%     Tvec = Tvec.*droneZaxis;
% 
%     p.vectors = quiver3(ax, rotorPts(1,:), rotorPts(2,:), rotorPts(3,:), ...
%                         Tvec(1,:), Tvec(2,:), Tvec(3,:));   
% end
