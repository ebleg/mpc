%% MAIN FILE
%
% Elke Salzmann & Emiel Legrand
% Delft University of Technology
%
% -------------------------------------------------------------------------

clear; clear positionMPC;
clc; close all;

addpath('..')
addpath('tools');
addpath('fun');
addpath('fun/mod');
addpath('fun/ctrl');

run parameters
run header

%% Define path to follow
% Parameterized for x,y,z with respect to t

fprintf('Objective trajectory: Ellipsoidal spiral\n') 
path = @(t) [4*cos(t); 4*sin(t); t/3];

% fprintf('\nObjective trajectory: Nondifferentiable 2D trajectory\n\n') 
% path = @(t) [t; 0*t; sign(t-2)+1]; 

% fprintf('Objective trajectory: Fly to a point,  2D\n') 
% path = @(t) [0*t; 0*t; 0*t+1]; % Fly straight up

%% Simulation initialization
sol = struct();
sol.t = (0:par.sim.h:par.sim.tmax);
nsteps = numel(sol.t);

sol.x.pos = nan(par.posCtrl.dim.x, nsteps);
sol.x.ang = nan(par.angCtrl.dim.x, nsteps);
sol.u.pos = nan(par.posCtrl.dim.u, nsteps);
sol.u.ang = nan(par.angCtrl.dim.u, nsteps);

%% Path & reference states
ref = generateReference(sol.t, path, par);

%% Set initial conditions
sol.x.pos(:,1) = ref.x.pos(:,1) + [0 0 0 0.2 0 0.2]';
sol.x.ang(:,1) = ref.x.ang(:,1);

%% Simulation loop
fprintf('Starting simulation loop...\n'); tic;
for i=2:(nsteps-par.posCtrl.dim.N)
    sol.u.pos(:,i) = positionMPC(sol.x.ang(:,i-1), ...
                                 sol.x.pos(:,i-1), ...
                                 sol.t(i), ...
                                 ref, par);
    sol.x.ang(:,i) = [zeros(3,1); sol.u.pos(2:3,i); ref.x.ang(6,i)];
    f = @(x) translationalDynamics(x, [sol.u.pos(:,i); ref.x.ang(6,i)] , par);
    sol.x.pos(:,i) = RK4(f, sol.x.pos(:,i-1), par.sim.h);
end
fprintf('Simulation ended - '); toc;
