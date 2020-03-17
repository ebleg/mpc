%% Test integration methods
% Comparison between GL4, RK4 and ode45

clear; clc; close all;
addpath('../tools');

% Equation to solve: Stiff van der Pol oscillator
dydt = @(y) [y(2); (1-y(1)^2)*y(2)-y(1)];
dydt_2 = @(t,y) [y(2); (1-y(1)^2)*y(2)-y(1)]; % ode45 expects t argument

h = 1e-2;
t = 0:h:20;
yGL4 = zeros(numel(t), 2);
yGL4(1,:) = [2; 0]; % Initial condition
options = optimoptions(@fsolve,'Display','off','SpecifyObjectiveGradient',false);

for i=2:numel(t)
    yGL4(i,:) = GL4(dydt, yGL4(i-1,:)', h, options);
end

[t2,y] = ode15s(dydt_2, [0 20], [2 0]');

figure
plot(t, yGL4(:,2), 'DisplayName', 'GL4');
hold on;
% plot(t, yRK4(1,:), 'DisplayName', 'RK4');
plot(t2, y(:,2), 'DisplayName', 'ode45');
legend();
grid on; grid minor;
ylabel('t');
xlabel('y1');
title('Numerical methods comparison for stiff van der Pol oscillator')
