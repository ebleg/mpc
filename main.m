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
addpath('fun/stability');

run parameters
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
sol.t.pos = (0:par.sim.h:par.sim.tmax);
sol.t.ang = (0:par.sim.h*(par.angCtrl.sampleInt/par.posCtrl.sampleInt):par.sim.tmax);
nsteps_pos = numel(sol.t.pos);
nsteps_ang = numel(sol.t.ang);

sol.x.pos = nan(par.posCtrl.dim.x, nsteps_pos);
sol.x.ang = nan(par.angCtrl.dim.x, nsteps_ang);
sol.u.pos = nan(par.posCtrl.dim.u, nsteps_pos);
sol.u.ang = nan(par.angCtrl.dim.u, nsteps_ang);

%% Path & reference states
ref = generateReference(sol.t.pos, path, par);

%% Set initial conditions
frame = par.posCtrl.sampleInt/par.angCtrl.sampleInt;
sol.x.pos(:,1) = ref.x.pos(:,1);%+ [0 0 0 0.2 0 0.2]';
sol.x.ang(:,1:frame) = ref_ang.x.ang(:,1:frame);
sol.x.pos(:,1) = ref.x.pos(:,1);%+ [0 0 0 0.2 0 0.2]';
sol.x.ang(:,1) = ref.x.ang(:,1);

predictionBuffer = ceil(par.posCtrl.dim.N*par.posCtrl.predInt/par.sim.h);
wdw = waitbar(0.02, sprintf('Simulation progress (%d)', 0.02*100));

predictionBufferPos = ceil(par.posCtrl.dim.N*par.posCtrl.predInt/par.sim.h);
predictionBufferAng = ceil(par.angCtrl.dim.N*par.angCtrl.predInt/par.sim.h);
predictionBuffer = max(predictionBufferPos, predictionBufferAng);
%% Simulation loop
fprintf('Starting simulation loop...\n'); tic;


% i=2:(nsteps-predictionBuffer)
% for i=2:50
%     sol.u.pos(:,i) = positionMPC(sol.x.ang(:,frame*(i-2)+10), ...
%                                  sol.x.pos(:,i-1), ...
%                                  sol.t.pos(i), ...
%                                  ref, par);
%     temp_u = nan(par.angCtrl.dim.u, frame+1);
%     temp_x = nan(par.angCtrl.dim.x, frame+1);
%     temp_x(:,1) = sol.x.ang(:,frame*(i-1));
%     for j=2:frame+1
%         disp([num2str(i), ', ' num2str(j)]);
%         % output MPC
% %         temp_u(:,j) = attitudeMPC([], par, sol.t.ang(frame*(i-2)+j), [],...
% %                                 ref_ang.x.ang(:,1),...
% %                                 ref_ang.x.ang(:,frame*(i-2)+j-1)); % Output MPC
%         % regular MPC
%         temp_u(:,j) = attitudeMPC(ref, par, sol.t.ang(frame*(i-2)+j), temp_x(:,j-1), [],[]);
%         g = @(x) rotationalDynamics(x, [sol.u.pos(1,i); temp_u(:,j)] , par);
%         temp_x(:,j) = GL4(g, temp_x(:,j-1), par);
%     end
%     sol.u.ang(:,frame*(i-2)+2:frame*(i-2)+11) = temp_u(:,2:frame+1);
%     sol.x.ang(:,frame*(i-1)+1:frame*(i-1)+10) = temp_x(:,2:frame+1); 
%     f = @(x) translationalDynamics(x, [sol.u.pos(:,i); sol.x.ang(6,frame*i)] , par);
%     sol.x.pos(:,i) = GL4(f, sol.x.pos(:,i-1), par);
% end

fprintf('Done - '); toc;
delete(wdw);

%% Visualisation
sol.x.pos = repelem(sol.x.pos,1,10);
sol.x.pos = sol.x.pos(:,1:nsteps_ang);
sol.u.pos = repelem(sol.u.pos,1,10);
sol.u.pos = sol.u.pos(:,1:nsteps_ang);

close all;
figure; ax = gca; axis equal; grid; grid minor; hold on;
title('Quadcopter simulation'); xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
% refPlot = plotTrajectory(ax, ref.t, ref.x.pos, '.', 'Reference trajectory');
% refPlot = plotTrajectory(ax, ref_pos.t.pos, ref_pos.x.pos, '.', 'Reference trajectory');
solPlot = plotTrajectory(ax, sol.t.ang, sol.x.pos, '.', 'Simulated trajectory');
legend();
simulateDrone(ax, sol, par);

profile viewer
