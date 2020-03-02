%% MODEL VERIFICATION

tspan = 0:1e-2:10;
hoverOmega = sqrt(param.drone.m*param.env.g/param.drone.rotor.Kf/4);

%% Freefall
omega = timeseries(zeros(4, numel(tspan)), tspan);
simOut = sim('quadcopter.slx');
pos_freefall = simOut.get('pos');

subplot(221)
plot(pos_freefall.Time, pos_freefall.Data(:, 3))
title('Freefall');
xlabel('t [s]')
ylabel('z [m]')
grid on; grid minor;

%% Hover
omega = timeseries(ones(4, numel(tspan))*hoverOmega, tspan);
simOut = sim('quadcopter.slx');
pos_hover = simOut.get('pos');

subplot(222)
plot(pos_hover.Time, pos_hover.Data(:, 3));
title('Hover');
xlabel('t [s]')
ylabel('z [m]')
grid on; grid minor;

%% Forward flight
omega = timeseries(ones(4, numel(tspan))*hoverOmega, tspan);
omega.Data(3, :, :) = omega.Data(3, :, :)*1.01;
omega.Data(4, :, :) = omega.Data(4, :, :)*1.01;
% omega.Data(1, :, :) = omega.Data(1, :, :);
% omega.Data(2, :, :) = omega.Data(2, :, :);

simOut = sim('quadcopter.slx');
pos_freefall = simOut.get('pos');

subplot(223)
plot3(pos_freefall.Data(:, 1), pos_freefall.Data(:, 2), pos_freefall.Data(:, 3));

title('Forward flight');
xlabel('t [s]')
ylabel('z [m]')
grid on; grid minor;
axis equal;