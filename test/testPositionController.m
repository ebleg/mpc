%% MAIN FILE
%
% Elke Salzmann & Emiel Legrand
% Delft University of Technology
%
% -------------------------------------------------------------------------

clear; clc;
close all;

addpath('..')
addpath('tools');
addpath('fun');
addpath('fun/mod');
addpath('fun/ctrl');

run parameters
par.sim.h = 1e-2;
par.sim.tmax = 4;

%% Define path to follow
% Parameterized for x,y,z with respect to t
% path = @(t) [4*cos(t); 4*sin(t); t/3]; % Ellipsoidal spiral
path = @(t) [t; 0*t; sign(t - 2) + 1];


%% Define initial conditions

%% Simulation initialization
t = (0:par.sim.h:par.sim.tmax);
nsteps = numel(t);
x_pos = nan(par.posCtrl.dim.x, nsteps);
x_ang = nan(6, nsteps); % no par entry yet

%% Path & reference states
pathpts = path(t);
[ref_pos, ref_ang, ref_u] = generateRefStates(pathpts, par);
ref_u_pos = [ref_u(1,:); ref_ang(4:5,:)];

x_pos(:,1) = ref_pos(:,1);
x_ang(:,1) = ref_ang(:,1);


%% Model verification
% Open loop simulation with reference inputs of the linear model and the
% nonlinear model
% linearValidation = nan(par.posCtrl.dim.x, nsteps);
% nonlinearValidation = nan(par.posCtrl.dim.x, nsteps);
% linearValidation(:,1) = ref_pos(:,1);
% nonlinearValidation(:,1) = ref_pos(:,1);
% 
% for i=2:(nsteps-par.posCtrl.dim.N)
%     fnl = @(x) translationalDynamics(x, [ref_u_pos(:,i); ref_ang(6,i)] , par);
%     flin = @(x) linearTranslationModel(x, ref_u_pos(:,i), ...
%                                        [par.drone.m*par.env.g; 0;0;ref_ang(6,i)], par);
%     nonlinearValidation(:,i) = RK4(fnl, nonlinearValidation(:,i-1), par.sim.h);
%     linearValidation(:,i) = RK4(flin, linearValidation(:,i-1), par.sim.h);
% end
% %%
% close all;
% plot3(pathpts(1,:), pathpts(2,:), pathpts(3,:), 'DisplayName', 'Reference path');
% hold on;
% plot3(nonlinearValidation(4,:), nonlinearValidation(5,:), nonlinearValidation(6,:), 'DisplayName', 'Nonlinear solution');
% plot3(linearValidation(4,:), linearValidation(5,:), linearValidation(6,:), 'DisplayName', 'Linear solution');

u_pos = nan(par.posCtrl.dim.u, nsteps);

%% Simulation loop
for i=2:(nsteps-par.posCtrl.dim.N)
    u_pos(:,i) = positionMPC(x_ang(:,i-1), ...
                        x_pos(:,i-1), ...
                        ref_u_pos(:,i:(i+par.posCtrl.dim.N-1)), ...
                        ref_pos(:,i:(i+par.posCtrl.dim.N)), ...
                        par);
    x_ang(:,i) = [zeros(3,1); u_pos(2:3,i); ref_ang(6,i)];
    f = @(x) translationalDynamics(x, [u_pos(:,i); ref_ang(6,i)] , par);
    x_pos(:,i) = RK4(f, x_pos(:,i-1), par.sim.h);
end

%%
close all;
plot3(pathpts(1,:), pathpts(2,:), pathpts(3,:));
xlabel('x')
ylabel('y')
hold on;
quivers = zeros(size(pathpts));
ddr = diff(pathpts, 2, 2)/par.sim.h/par.sim.h;
ddr = [ddr ddr(:,end) ddr(:,end)]; % Acceleration along path

for i = 1:numel(t)
    R = eul2rotm(ref_ang(4:6,i)', 'XYZ');
    quivers(:,i) = R*[0; 0; 1]*norm(ddr(:,i));
end

quiver3(pathpts(1,:), pathpts(2,:), pathpts(3,:), quivers(1,:), quivers(2,:), quivers(3,:))

hold on;

plot3(x_pos(4,:), x_pos(5,:), x_pos(6,:));

% function dx = linearTranslationModel(x, u, setpt, par)
%     LTI = simpTranslationalDynamics(setpt, par);
%     fcn = @(e) translationalDynamics([0 0 5 0 0 0]', e, par);
%     B = jacobianest(fcn, setpt);
%     B = B(:,1:3);
%     dx = LTI.A*x + LTI.B*(u - setpt(1:3));
% end