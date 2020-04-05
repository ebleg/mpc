function u_opt = attitudeControl(LTI, xref, par, att) %, x0
%% Attitude controller
% x_ang: state vector for rotational dynamics, i.e. [phidot thetadot psidot phi theta psi]'
% u_ang: input vector for rotational dynamics, i.e. [U2 U3 U4];

%% Stability analysis
controllability(LTI)
     
%     [P,L,G] = idare(A_d,B_d,Q,R);
%     
%     A_K = A_d-B_d*G;

%% Regular MPC    
%     att = sol.x.ang(:,1);
%     LTI = par.angCtrl.LTI;
N = par.angCtrl.dim.N;
    
% prediction matrix
[T,S] = predmodgen(LTI, par.angCtrl.dim);

% cost function
Qbar = blkdiag(kron(eye(N), par.angCtrl.Q), par.angCtrl.P);
Rbar = kron(eye(N), par.angCtrl.R);

xref = xref(1:(N+1)*par.angCtrl.dim.x)';

H = S'*Qbar*S + Rbar;
h = S'*Qbar*T*att + S'*Qbar*xref; %+Rbar'*uref
    
% Optimization
cvx_begin quiet
    variable u_opt(par.angCtrl.dim.u*N)
    minimize(0.5*u_opt'*H*u_opt+h'*u_opt)
    subject to
    % input contraints
    par.angCtrl.F*u_opt <= par.angCtrl.f;
%     F * u_N <= a_lim
%     % state constraints
%     S*u_N <= -T*att + x_lim;
%     S*u_N >= -T*att - x_lim;
cvx_end

u_opt = u_opt(1:par.angCtrl.dim.u);

end
