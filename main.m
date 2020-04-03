 %% MAIN FILE
%
% Elke Salzmann & Emiel Legrand
% Delft University of Technology
%
% -------------------------------------------------------------------------

clear; clear positionMPC; clear attitudeMPC;
clc; close all;

profile on

addpath('..')
addpath('tools');
addpath('fun');
addpath('fun/mod');
addpath('fun/ctrl');
addpath('fun/vis');

run parameters
run header

%% Define path to follow
% Parameterized for x,y,z with respect to t

fprintf('Objective trajectory: Ellipsoidal spiral\n') 
path = @(t) [4*cos(t); 4*sin(t); t/3];

% fprintf('\nObjective trajectory: RRT* generated path\n\n') 
% load('shapes.mat');
% rrtpath = load('path.mat');
% rrtpath = rrtpath.path;
% load('nodes.mat');
% par.sim.tmax = 20;
% 
% path = @(t) piecewiseLinearPath(nodes, rrtpath, par, t);

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
sol.x.pos(:,1) = ref.x.pos(:,1);%+ [0 0 0 0.2 0 0.2]';
sol.x.ang(:,1) = ref.x.ang(:,1);

predictionBufferPos = ceil(par.posCtrl.dim.N*par.posCtrl.predInt/par.sim.h);
predictionBufferAng = ceil(par.angCtrl.dim.N*par.angCtrl.predInt/par.sim.h);

predictionBuffer = max(predictionBufferPos, predictionBufferAng);
%% Simulation loop
fprintf('Starting simulation loop...\n'); tic;
for i=2:50
    sol.u.pos(:,i) = positionMPC(sol.x.ang(:,i-1), ...
                                 sol.x.pos(:,i-1), ...
                                 sol.t(i), ...
                                 ref, par);
    sol.u.ang(:,i) = attitudeMPC(ref, par, sol.t(i), sol.x.ang(:,i-1), [],[]);
	omega = -sol.u.ang(3,i)/par.drone.rotor.Km;
%     sol.x.ang(:,i) = [zeros(3,1); sol.u.pos(2:3,i); ref.x.ang(6,i)];
    sol.x.ang(:,i) = rotationalDynamics([sol.u.pos(2:3,i); sol.x.ang(3:6,i-1)],...
                                        [sol.u.ang(:,i); omega], par);
    f = @(x) translationalDynamics(x, [sol.u.pos(:,i); sol.x.ang(6,i)] , par);
%     f = @(x) translationalDynamics(x, [sol.u.pos(:,i); ref.x.ang(6,i)] , par);
    sol.x.pos(:,i) = GL4(f, sol.x.pos(:,i-1), par);
end
fprintf('Simulation ended - '); toc;

%% Visualisation
close all;
figure; ax = gca; axis equal; grid; grid minor; hold on;
title('Quadcopter simulation'); xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
% refPlot = plotTrajectory(ax, ref.t, ref.x.pos, '.', 'Reference trajectory');
refPlot = plotTrajectory(ax, ref.t, ref.x.pos, '.', 'Reference trajectory');
solPlot = plotTrajectory(ax, sol.t, sol.x.pos, '.', 'Simulated trajectory');
legend();
simulateDrone(ax, sol, par);

profile viewer
