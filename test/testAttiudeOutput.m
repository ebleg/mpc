clear; clear attitudeMPC; clear positionMPC; clear attitudeOutputMPC;

%% Initialization file
addpath('..');
addpath('../fun');
addpath('../fun/ctrl');
addpath('../fun/mod');
addpath('../fun/vis');
addpath('../tools');
run parameters
run initLTI

%% Make path and references
path = @(t) [4*cos(t); 4*sin(t); t/3]; %Ellipsoidal spiral

sol = struct();
sol.t = (0:par.sim.h:par.sim.tmax);
nsteps = numel(sol.t);

sol.x.pos = nan(par.posCtrl.dim.x, nsteps);
sol.x.ang = nan(par.angCtrl.dim.x, nsteps);
sol.u.pos = nan(par.posCtrl.dim.u, nsteps);
sol.u.ang = nan(par.angCtrl.dim.u, nsteps);

ref = generateReference(sol.t, path, par);

% sol.x.ang = ref.x.ang;
% sol.u.ang = ref.u.ang;
sol.u.pos = ref.u.pos;
sol.x.pos = ref.x.pos;
sol.x.ang(:,1) = ref.x.ang(:,1);
ref.y.ang = LTI.C*ref.x.ang;

for i = 1:10
    disp(num2str(i))
    yref = ref.y.ang(:,i);
    [x, u, x_0, xehat_0, e] = attitudeOutputControl(LTI, LTI_e, par, yref, pred, x_1, xehat_1);
    sol.x.ang(:,i) = x;
    sol.u.ang(:,i) = u;
    error(:,i) = e;
    x_1 = x_0;
    xehat_1 = xehat_0;
end

plot(error(1,:),'b')
hold on
plot(error(2,:),'r')
plot(error(3,:),'y')
plot(error(4,:),'g')
plot(error(5,:),'m')
plot(error(6,:),'c')
title('Offset free output MPC'); ylabel('Error'); xlabel('Iteration');

