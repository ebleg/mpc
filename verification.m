%% MODEL VERIFICATION

tspan = 0:1e-2:10;
hoverOmega = sqrt(param.drone.m*param.env.g/param.drone.rotor.Kf/4);

model = 'quadcopter_plant_only.slx';

%% Freefall
omega = timeseries(zeros(4, numel(tspan)), tspan);
simOut = sim(model);
pos_freefall = simOut.get('pos');

subplot(221)
plot(pos_freefall.Time, pos_freefall.Data(:, 3))
title('Freefall');
xlabel('t [s]')
ylabel('z [m]')
grid on; grid minor;

%% Hover
omega = timeseries(ones(4, numel(tspan))*hoverOmega, tspan);
simOut = sim(model);
pos_hover = simOut.get('pos');

subplot(222)
plot(pos_hover.Time, pos_hover.Data(:, 3));
title('Hover');
xlabel('t [s]')
ylabel('z [m]')
grid on; grid minor;

% Wobble around y-axis
omega = timeseries(ones(numel(tspan), 4)*hoverOmega, tspan');
omega.Data(:, 1) = omega.Data(:, 1).*(1 + 0.005*sin(2*tspan'));
omega.Data(:, 3) = omega.Data(:, 3).*(1 - 0.005*sin(2*tspan'));

simOut = sim(model);
pos_wobble = simOut.get('pos');
att_wobble = simOut.get('att');

subplot(223)
hold on;
plot(tspan, omega.Data(:, 3), 'DisplayName', '\omega_3')
plot(tspan, omega.Data(:, 1), 'DisplayName', '\omega_1')

yyaxis right;
plot(att_wobble.Time, att_wobble.Data(:, 2), 'DisplayName', '\theta')
legend()

title('Wobble around y-axis');
xlabel('t [s]')
ylabel('z [m]')
grid on; grid minor;