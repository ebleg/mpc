% Calculates output of attitude controller using offset-free output MPC
clear;

%% Initialization file
addpath('..');
addpath('../fun');
addpath('../fun/ctrl');
addpath('../fun/mod');
addpath('../fun/vis');
addpath('../tools');
run parameters

dim = par.angCtrl.dim;

sol = struct();                        
sol.t = (0:par.sim.h:par.sim.tmax);
nsteps = numel(sol.t);

path = @(t) [4*cos(t); 4*sin(t); t/3]; %Ellipsoidal spiral

sol.x.pos = nan(par.posCtrl.dim.x, nsteps);
sol.x.ang = nan(par.angCtrl.dim.x, nsteps);
sol.u.pos = nan(par.posCtrl.dim.u, nsteps);
sol.u.ang = nan(par.angCtrl.dim.u, nsteps);

ref = generateReference(sol.t, path, par);

sol.x.pos = ref.x.pos;
sol.x.ang = ref.x.ang;
sol.u.pos = ref.u.pos;
sol.u.ang = ref.u.ang;

run initLTI
dist = LTI.d;   

%% Optimization
x(:,1) = LTI.x0;
xe(:,1) = [x(:,1);dist];
xehat(:,1) = [ref.x.ang(:,1); LTI.d]';
yref = LTI.C*ref.x.ang;
uref = ref.u.ang;

simTime = 10;

for i=1:simTime
    % Observer
    disp(num2str(i))
    xhat = xehat(1:dim.x,i);
    dhat = xehat(dim.x+1:end,i);

    % Target Selector
    A_output = [eye(dim.x)-LTI.A -LTI.B ;LTI.C, zeros(dim.x, dim.u)];
    b_output = [LTI.Bd*dhat; yref(:,i)-LTI.Cd*dhat];

    H_OTS = blkdiag(zeros(dim.x),eye(dim.u));
    h_OTS = zeros(dim.x+dim.u,1);

    cvx_begin quiet
        variable x_r_u_r(dim.x+dim.u)
        minimize (1/2*quad_form(x_r_u_r,H_OTS) + h_OTS'*x_r_u_r)
        subject to
        A_output * x_r_u_r <= b_output;
    cvx_end

    x_r = x_r_u_r(1:dim.x);
    u_r = x_r_u_r(dim.x+1:end);

    x_tilde = x_r - xhat;
    h_e = (x_tilde'*pred.T'*pred.Qbar*pred.S)' ...
                - pred.S'*pred.Qbar*kron(ones(dim.N+1,1),eye(dim.x))*x_r...
                - pred.Rbar*kron(ones(dim.N,1),eye(dim.u))*u_r;

    cvx_begin quiet
        variable u_N(dim.u*dim.N)
        minimize ( (1/2)*quad_form(u_N,pred.H_e) + (h_e'*u_N ))
        subject to
            % input contraints
            par.angCtrl.F*u_N <= par.angCtrl.f;
    cvx_end    
    u_opt = u_N(1:dim.u);
%     sol.u.ang(1:dim.u,i) = u_opt;
%     u_opt = ref.u.ang(:,i);

    % Real system
    x(:,i+1) = LTI.A*x(:,i) + LTI.B*u_opt + LTI.Bd*dist;
    y(:,i) = LTI.C*x(:,i) + LTI.Cd*dist;

    % Observer
    yhat(:,i)=LTI_e.C*xehat(:,i);
    xehat(:,i+1)=LTI_e.A*xehat(:,i)+LTI_e.B*u_opt+pred.G*(y(:,i)-yhat(:,i));
    
    error(:,i) = x(:,i) - xhat;

    u = u_opt;

%     Vf(i) = 0.5*x(:,i)'* par.angCtrl.P*x(:,i);
%     l(i) = 0.5*x(:,i)'*par.angCtrl.Q*x(:,i);
end

close all;
plot(error(1,:),'b')
hold on
plot(error(2,:),'r')
plot(error(3,:),'y')
plot(error(4,:),'g')
plot(error(5,:),'m')
plot(error(6,:),'c')
title('Offset free output MPC'); ylabel('Error'); xlabel('Iteration');

% close all;
% plot(error_u(1,:),'b')
% hold on
% plot(error_u(2,:),'r')
% plot(error_u(3,:),'y')
% title('Offset free output MPC'); ylabel('Error output'); xlabel('Iteration');