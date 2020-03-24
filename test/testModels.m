%% Model verification
% Hover 
run parameters
close all;

mg = par.drone.m*par.env.g;

x0 = [0 0 0 0 0 0]';
u = @(t) [(1 + 0.4*sin(t))*mg; t*0; t*0];
f_nl = @(t, x) translationalDynamics(x, [u(t); 0], par);
LTI = simpTranslationalDynamics([mg; 0; 0; 0], par);
LTI.B(3,1) = 1;
f_lin = @(t, x) LTI.A*x + LTI.B*(u(t) - [mg 0 0]');
[t_nl, x_nl] = ode15s(f_nl, [0 10], x0);
[t_lin, x_lin] = ode15s(f_lin, [0 10], x0); 

plot(t_nl, x_nl(:,6), 'DisplayName', 'Nonlinear solution');
hold on;
plot(t_lin, x_lin(:,6), 'DisplayName', 'Linear solution');
