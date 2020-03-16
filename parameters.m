%% PARAMETERS
%
% Elke Salzmann, Emiel Legrand
%
% -------------------------------------------------------------------------

par = struct();

%% Quadcopter properties
par.drone.Ixx = 7.5e-3;
par.drone.Iyy = 7.5e-3;
par.drone.Izz = 1.3e-2;
par.drone.I = diag([par.drone.Ixx par.drone.Iyy par.drone.Izz]);
par.drone.l = 0.23;
par.drone.m = 0.2; % ????????????????????? look up in doctoral thesis

par.drone.rotor.I = 6e-5;
par.drone.rotor.Kf = 3.13e-5;
par.drone.rotor.Km = 7.5e-7;

par.drone.Kt = diag([0.1 0.1 0.15]);

%% General parameters
par.env.g = 9.80665;

%% Initial conditions
par.x0.pos = [0 0 2]';
par.x0.att = [0 0 0]';
par.x0.linvel = [0 0 0]';
par.x0.angvel = [0 0 0]';

%% Shorthand parameters
% Stored for computational efficiency in simpRotationalDynamics and 
% simpTranslationalDynamics

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
par.posCtrl.Q = eye(par.posCtrl.dim.x);
par.posCtrl.R = eye(par.posCtrl.dim.u);
par.posCtrl.P = eye(par.posCtrl.dim.x); % Might be overwritten by DARE solution

% Sample rate
par.posCtrl.Ts = 0.1; % Hz

%% Position target selection parameters
par.posTarSel.Q = eye(par.posCtrl.dim.x);
par.posTarSel.R = eye(par.posCtrl.dim.u);

%% Optimization settings
par.opt.settings = sdpsettings('verbose', 0, ...
                               'solver', 'quadprog', ...
                               'quadprog.maxiter',100);