%% PARAMETERS
%
% Elke Salzmann, Emiel Legrand
%
% -------------------------------------------------------------------------

% This file contains all parameters for the MPC problem:
% - Drone physical properties
% - Weight matrices for all cost functions
% - Settings
% Some other values are precomputed because they are used at every timestep
% of the MPC controllers or integration of the system dynamics. 

par = struct();

%% Simulation parameters
par.sim.tmax = 2;
par.sim.h = 0.02; % ODE integration timestep

%% Quadcopter properties
par.drone.Ixx = 7.5e-3;
par.drone.Iyy = 7.5e-3;
par.drone.Izz = 1.3e-2;
par.drone.I = diag([par.drone.Ixx par.drone.Iyy par.drone.Izz]);
par.drone.l = 0.23;
par.drone.m = 0.650;
par.drone.rotor.I = 6e-5;
par.drone.rotor.Kf = 3.13e-5;
par.drone.rotor.Km = 7.5e-7;

par.drone.Kt = diag([0.1 0.1 0.15]);

% Matrix mapping between input and rotor speed SQUARED
par.drone.omega2u = [par.drone.rotor.Kf*[1 1 1 1; 0 -1 0 1; 1 0 -1 0];
                           par.drone.rotor.Km*[1 -1 1 -1]];
                       
par.drone.u2omega = par.drone.omega2u^-1; % precompute for efficiency 

%% General parameters
par.env.g = 9.80665;

%% Shorthand parameters
% Stored for computational efficiency in simpRotationalDynamics and 
% simpTranslationalDynamics
% [from ElKholy]

par.drone.a1 = (par.drone.Iyy - par.drone.Izz)./par.drone.Ixx;
par.drone.a2 = par.drone.rotor.I/par.drone.Ixx;
par.drone.a3 = (par.drone.Izz- par.drone.Ixx)/par.drone.Iyy;
par.drone.a4 = par.drone.rotor.I/par.drone.Iyy;
par.drone.a5 = (par.drone.Ixx - par.drone.Iyy)/par.drone.Izz;
par.drone.b1 = par.drone.l/par.drone.Ixx;
par.drone.b2 = par.drone.l/par.drone.Iyy;
par.drone.b3 = par.drone.l/par.drone.Izz;

par.drone.nomThrust = par.drone.m*par.env.g/4;

%% Input control parameters
par.cstr.maxVel = 285; % [rad/s], Include 85% SF margin to reduce wear on the gearbox

% 0.08 = Pel,max*eff/omega_max, 0.039 = k_T*omega_max^2
par.cstr.maxAcc = (0.08 - 0.039)/par.drone.rotor.I*0.85; % Seems reasonable

%% Position control parameters
% Problem dimensions
par.posCtrl.dim.u = 3; % Input vector length
par.posCtrl.dim.x = 6; % State vector length
par.posCtrl.dim.y = 6; % Assume full-state knowledge for now
par.posCtrl.dim.N = 5; % Prediction horizon

% Cost matrices
par.posCtrl.Q = diag([1 1 1 25 25 25]);
par.posCtrl.R = diag([.05 1. 1.]);
par.posCtrl.P = diag([1 1 1 50 50 50]); % Might be overwritten by DARE solution

% Sample rate
par.posCtrl.sampleInt = 6*par.sim.h;   % Position MPC sample rate
par.posCtrl.predInt = 6*par.sim.h;      % Position MPC prediction interval
[par.posCtrl.T, par.posCtrl.f] = posCstrMatrix(par);

% Terminal set
par.posCtrl.Xf = 0.5*ones(1,par.posCtrl.dim.x)*par.posCtrl.P*ones(par.posCtrl.dim.x,1);

%% Attitude control parameters
% Problem dimensions
par.angCtrl.dim.u = 3; % Input vector length
par.angCtrl.dim.x = 6; % State vector length
par.angCtrl.dim.y = 6; % Assume full-state knowledge for now
par.angCtrl.dim.N = 8; % Prediction horizon

% Sample rate
% par.angCtrl.sampleInt = par.posCtrl.sampleInt/10;   % Position MPC sample rate; should be at least 10 times smaller than the sample rate for the position control and a divisor of the sample rate for the position control
% par.angCtrl.predInt = par.angCtrl.sampleInt;      % Position MPC prediction interval
par.angCtrl.sampleInt = par.sim.h;   % Position MPC sample rate; should be at least 10 times smaller than the sample rate for the position control and a divisor of the sample rate for the position control
par.angCtrl.predInt = par.sim.h;      % Position MPC prediction interval


% System
par.angCtrl.LTI = c2d(simpRotationalDynamics(par, [0 0 0 0 0 0]'), par.angCtrl.sampleInt, 'zoh'); % Linear system around hover

% Cost matrices
par.angCtrl.Q = eye(par.angCtrl.dim.x)*diag([1 1 1 20 20 20]);%*diag([1 1 1 20 20 20]);
par.angCtrl.R = eye(par.angCtrl.dim.u)*diag([0.1 0.1 0.1]);
par.angCtrl.P =  dare(par.angCtrl.LTI.A, par.angCtrl.LTI.B, par.angCtrl.Q, par.angCtrl.R);

% Constraints
[par.angCtrl.F, par.angCtrl.f] = attCstrMatrix(par);
par.angCtrl.x_lim = 0.5;

%% fsolve options
par.settings.solve = optimoptions(@fsolve, 'Display', 'none');
par.settings.opts = optimoptions('quadprog','Display','off');