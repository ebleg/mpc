clear

addpath('..');
addpath('../fun');
addpath('../fun/ctrl');
addpath('../fun/mod');
addpath('../tools')
run parameters

t = 0:pi/100:pi;
phi = 5*cos(t);
theta = 5*sin(t);
psi = t;
% plot3(phi,theta,psi,'*r');
% for i=1:length(phi)
%   plot3(phi(i),theta(i),psi(i),'*r');
%   hold on;
%   pause(0.01);
% end

xref=zeros(par.angCtrl.dim.x, par.angCtrl.dim.N+1);
for i=1:par.angCtrl.dim.N+1
    xref(:,i) = [1 1 1 phi(i) theta(i) psi(i)]';
end

att = xref(:,1);

u_input = [1 1 1]';
uref=zeros(par.angCtrl.dim.u, par.angCtrl.dim.N);
for i=1:par.angCtrl.dim.N
    uref(:,i) = u_input;
end

[LTI_rot_c] = simpRotationalDynamics(par, xref);
[LTI_rot_d] = c2d(LTI_rot_c, par.angCtrl.sampleInt, 'zoh');
[u, LTI] = attitudeMPC(LTI_rot_d, xref, uref, par, att);