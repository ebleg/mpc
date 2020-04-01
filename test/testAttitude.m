clear

addpath('..');
addpath('../fun');
addpath('../fun/ctrl');
addpath('../fun/mod');
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

MPC = 0; %0 for regulation MPC, 1 for output MPC

par.sim.tmax = 20;
path = @(t) [0*t 0*t t]; % Fly straight up

sol = struct();
sol.t = (0:par.sim.h:par.sim.tmax);
nsteps = numel(sol.t);

sol.x.pos = nan(par.posCtrl.dim.x, nsteps);
sol.x.ang = nan(par.angCtrl.dim.x, nsteps);
sol.u.pos = nan(par.posCtrl.dim.u, nsteps);
sol.u.ang = nan(par.angCtrl.dim.u, nsteps);

ref = generateReference(sol.t, path, par);

sol.x.pos(:,1) = ref.x.pos(:,1);%+ [0 0 0 0.2 0 0.2]';
sol.x.ang(:,1) = ref.x.ang(:,1);

predictionBuffer = ceil(par.angCtrl.dim.N*par.angCtrl.predInt/par.sim.h);

for i=2:(nsteps-predictionBuffer)
    sol.u.ang(:,i) = attitudeMPC(MPC, ref, par, sol.x.ang(:,i-1), [],[]);
%     sol.x.ang(:,i) = [zeros(3,1); sol.u.pos(2:3,i); ref.x.ang(6,i)];
%     f = @(x) translationalDynamics(x, [sol.u.pos(:,i); ref.x.ang(6,i)] , par);
%     sol.x.pos(:,i) = RK4(f, sol.x.pos(:,i-1), par.sim.h);
end