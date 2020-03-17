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
par.sim.h = 1e-1;

%% Define path to follow
% Parameterized for x,y,z with respect to t
path = @(t) [1*cos(1*t); 5*sin(1*t); t/3]; % Ellipsoidal spiral

%% Define initial conditions
pos0 = zeros(6,1);
ang0 = zeros(6,1);

%% Simulation initialization
t = (0:par.sim.h:par.sim.tmax);
nsteps = numel(t);
x_pos = nan(par.posCtrl.dim.x, nsteps);
x_pos(:,1) = pos0;
x_ang = nan(6, nsteps); % no par entry yet
x_ang(:,1) = ang0;

%% Path & reference states
pathpts = path(t);
[ref_pos, ref_ang, ref_u] = generateRefStates(pathpts, par);

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