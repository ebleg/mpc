%% PARAMETERS
%
% Emiel Legrand
% 01/03/2020
%
% -------------------------------------------------------------------------

param = struct();

%% Quadcopter properties
param.drone.Ixx = 7.5e-3;
param.drone.Iyy = 7.5e-3;
param.drone.Izz = 1.3e-2;
param.drone.I = diag([param.drone.Ixx param.drone.Iyy param.drone.Izz]);
param.drone.l = 0.23;
param.drone.m = 0.2; % ????????????????????? look up in doctoral thesis

param.drone.rotor.J = 6e-5;
param.drone.rotor.Kf = 3.13e-5;
param.drone.rotor.Km = 7.5e-7;

param.drone.Kt = diag([0.1 0.1 0.15]);

%% General parameters
param.env.g = 9.80665;

%% Initial conditions
param.x0.pos = [0 0 2]';
param.x0.att = [0 0 0]';
param.x0.linvel = [0 0 0]';
param.x0.angvel = [0 0 0]';

param.x0.angvel = [0 0 0]';

%% Rotation matrix


