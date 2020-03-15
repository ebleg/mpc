%% MAIN FILE
%
% Elke Salzmann & Emiel Legrand
% Delft University of Technology
%
% -------------------------------------------------------------------------

clear; 
close all;

addpath('tools');
addpath('fun');
addpath('fun/mod');
addpath('fun/ctrl');
addpath('fun/obs');

run parameters

%% Define path to follow
% Parameterized for x,y,z with respect to t
path = @(t) [3*cos(t); 1*sin(t); t/3]; % Ellipsoidal spiral

t = 0:0.01:10;
pathpts = path(t);
plot3(pathpts(1,:), pathpts(2,:), pathpts(3,:))
grid; grid minor;

%% Define initial conditions


%% 