%% MAIN FILE
%
% Elke Salzmann & Emiel Legrand
% Delft University of Technology
%
% -------------------------------------------------------------------------

clear; clear positionMPC; clear attitudeMPC;
clc; close all;

addpath('..')
addpath('tools');
addpath('fun');
addpath('fun/mod');
addpath('fun/ctrl');
addpath('fun/vis');
addpath('fun/stability');

run parameters
run initLTI
run header

%% Define path to follow
% Parameterized for x,y,z with respect to t

fprintf('Objective trajectory: Ellipsoidal spiral\n') 
path = @(t) [2*cos(t); 12*sin(t); t/3];

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

% fprintf('\nObjective trajectory: Nondifferentiable 2D trajectory\n\n') 
% path = @(t) [4*t; 0*t; sign(t-6)+1]; 

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
% frame = par.posCtrl.sampleInt/par.angCtrl.sampleInt;
sol.x.pos(:,1) = ref.x.pos(:,1);%+ [0 0 0 0.2 0 0.2]';
sol.x.ang(:,1) = ref.x.ang(:,1);
sol.x.pos(:,1) = ref.x.pos(:,1);%+ [0 0 0 0.2 0 0.2]';
sol.x.ang(:,1) = ref.x.ang(:,1);
yref(:,1) = LTI.C*ref.x.ang(:,1);

x_1 = LTI.x0;
xehat_1=[ref.x.ang(:,1); LTI.d];

% predictionBuffer = ceil(par.posCtrl.dim.N*par.posCtrl.predInt/par.sim.h);
wdw = waitbar(0.02, sprintf('Simulation progress (%d)', 0.02*100));

predictionBufferPos = ceil(par.posCtrl.dim.N*par.posCtrl.predInt/par.sim.h);
predictionBufferAng = ceil(par.angCtrl.dim.N*par.angCtrl.predInt/par.sim.h);
predictionBuffer = max(predictionBufferPos, predictionBufferAng);
%% Simulation loop
fprintf('Starting simulation loop...\n'); tic;

% i=2:(nsteps-predictionBuffer)
for i=2:(nsteps-predictionBuffer)
    disp(num2str(i))
    sol.u.pos(:,i) = positionMPC(sol.x.ang(:,i-1), ...
                                 sol.x.pos(:,i-1), ...
                                 sol.t(i), ...
                                 ref, par);
    [u, x_0, xehat_0, e] = attitudeMPC(LTI, LTI_e, par, yref(:,i-1), pred, x_1, xehat_1, sol.t(i));
    x_1 = x_0; xehat_1 = xehat_0; sol.u.ang(:,i) = u; 
    sol.u.ang(:,i) = ref.u.ang(:,i);
    g = @(x) rotationalDynamics(x, [sol.u.pos(1,i); sol.u.ang(:,i)] , par);
    sol.x.ang(:,i) = GL4(g, sol.x.ang(:,i-1), par);
    f = @(x) translationalDynamics(x, [sol.u.pos(:,i); sol.x.ang(6,i)] , par);
    sol.x.pos(:,i) = GL4(f, sol.x.pos(:,i-1), par);
    yref(:,i) = LTI.C * sol.x.ang(:,i);
end
fprintf('Done - '); toc;
delete(wdw);

%% Visualisation
close all;
figure; ax = gca; axis equal; grid; grid minor; hold on;
title('Quadcopter simulation'); xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
% refPlot = plotTrajectory(ax, ref.t, ref.x.pos, '.', 'Reference trajectory');
% refPlot = plotTrajectory(ax, ref.t.pos, refs.x.pos, '.', 'Reference trajectory');
solPlot = plotTrajectory(ax, sol.t, sol.x.pos, '.', 'Simulated trajectory');
legend();
simulateDrone(ax, sol, par);
