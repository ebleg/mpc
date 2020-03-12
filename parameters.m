%% PARAMETERS
%
% Emiel Legrand
% 01/03/2020
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

par.drone.rotor.J = 6e-5;
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
par.drone.a2 = par.drone.rotor.J/par.drone.Ixx;
par.drone.a3 = (par.drone.Izz- par.drone.Ixx)/par.drone.Iyy;
par.drone.a4 = par.drone.rotor.J/par.drone.Iyy;
par.drone.a5 = (par.drone.Ixx - par.drone.Iyy)/par.drone.Izz;
par.drone.b1 = par.drone.l/par.drone.Ixx;
par.drone.b2 = par.drone.l/par.drone.Iyy;
par.drone.b3 = par.drone.l/par.drone.Izz;

par.drone.nomThrust = par.drone.m*par.env.g/4;




