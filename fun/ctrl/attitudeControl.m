function [u_opt] = attitudeControl(LTI, xref, uref, par, att)
%% Attitude controller
% x_ang: state vector for rotational dynamics, i.e. [p q r phi theta psi]'
% u_ang: input vector for rotational dynamics, i.e. [u1 u2 u3 u4]

%% Stability analysis
%     A_dis = c2d(LTI_rot.A, par.attCtrl.Ts, 'zoh');
%     B_dis = c2d(LTI_rot.B, par.attCtrl.Ts, 'zoh');
% 
%     controllability(LTI)
% 
%     Q = eye(par.dim.x);
%     R = eye(par.dim.u);
%     P = eye(par.dim.x);
%     
%     [X,L,K_LQR] = idare(A_dis,B_dis,Q,R);
%     
%     A_K = A_dis-B_dis*K_LQR;

%% Regulation MPC    
%     att = sol.x.ang(:,i-1);
%     LTI = LTI_rot_d;
    
% prediction matrix
[T,S] = predmodgen(LTI, par.angCtrl.dim);

% cost function
Qbar = blkdiag(kron(eye(par.angCtrl.dim.N), par.angCtrl.Q), par.angCtrl.P);
Rbar = kron(eye(par.angCtrl.dim.N), par.angCtrl.R);

uref = uref(1: par.angCtrl.dim.N*par.angCtrl.dim.u)';
xref = xref(1:(par.angCtrl.dim.N+1)*par.angCtrl.dim.x)';

v_lim = par.cstr.maxVel;
a_lim = par.cstr.maxAcc;

H = S'*Qbar*S + Rbar;
% h = att'*T'*Qbar*S - xref'*Qbar*S;% - Rbar'*uref;
h = Rbar'*uref + S'*Qbar*T*att + S'*Qbar*xref;

% Optimization
N = par.angCtrl.dim.N;
cvx_begin 
    variable u_opt(par.angCtrl.dim.u*N)
    minimize(0.5*quad_form(u_opt,H)+h'*u_opt)
%     subject to
%     u_opt <= v_lim *ones(4*N,1);
cvx_end

[u_opt] = u_opt(1:par.angCtrl.dim.u);
    

end
