function [u, x_1, xehat_1, e] = attitudeOutputControl(LTI, LTI_e, par, uref, yref, pred, x_1, xehat_1)
% Calculates output of attitude controller using offset-free output MPC

dim = par.angCtrl.dim;

dist = LTI.d;                           % Dit kan moeilijker gemaakt worden

Q = par.angCtrl.Q;
R = par.angCtrl.R;
P = par.angCtrl.P;

% LTI.C = zeros(dim.y,dim.x);  %measured outputs
% LTI.C(1,4) = 1;
% LTI.C(2,5) = 1;
% LTI.C(3,6) = 1;

%% Optimization

% Output
x = x_1;
y = LTI.C*x;
    
% Observer
xehat = xehat_1;
xhat = xehat(1:dim.x);
dhat = xehat(dim.x+1:end);

% Target Selector
A_output = [eye(dim.x)-LTI.A -LTI.B ;LTI.C, zeros(dim.x, dim.u)];
b_output = [LTI.Bd*dhat; yref-LTI.Cd*dhat];

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
u_tilde = u_r - uref;
h_e = (x_tilde'*pred.T'*pred.Qbar*pred.S)'-...
        pred.S'*pred.Qbar*kron(ones(dim.N+1,1),eye(dim.x))*x_r -...
        pred.Rbar*kron(ones(dim.N,1),eye(dim.u))*u_r;

cvx_begin quiet
    variable u_N(dim.u*dim.N)
    minimize ( (1/2)*quad_form(u_N,pred.H_e) + (h_e'*u_N ))
    subject to
    % input contraints
    par.angCtrl.F*(u_N - repmat(u_r,dim.N,1)) <= par.angCtrl.f;
cvx_end    
u_opt = u_r +u_N(1:dim.u);

% Real system
x_1 = LTI.A*x + LTI.B*u_opt + LTI.Bd*dist;
y = LTI.C*x + LTI.Cd*dist;

% Observer
yhat=LTI_e.C*xehat;
xehat_1=LTI_e.A*xehat+LTI_e.B*u_opt+pred.G*(y-yhat);

e = x - xhat;
u = u_opt;


