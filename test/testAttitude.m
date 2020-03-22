% x = [p q r phi theta psi]';
% u = [u1 u2 u3 u4]';

t = 0:pi/100:pi;
phi = 5*cos(t);
theta = 5*sin(t);
psi = t;
% plot3(phi,theta,psi,'*r');

for i=1:N+1
    xref = [1 1 1 phi(i) theta(i) psi(i)];
end
% for i=1:length(phi)
%   plot3(phi(i),theta(i),psi(i),'*r');
%   hold on;
%   pause(0.01);
% end

[u] = attitudeMPC(LTI, xref, uref);