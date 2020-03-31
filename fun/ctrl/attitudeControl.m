function [u] = attitudeControl(LTI, xref, uref, par, att)
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
%     par.attCtrl.dim.u = 4;
%     par.attCtrl.dim.x = 6;
%     par.attCtrl.dim.N = 10;
%     
%     par.attCtrl.Q = eye(par.attCtrl.dim.x);
%     par.attCtrl.R = eye(par.attCtrl.dim.u);
%     par.attCtrl.P = eye(par.attCtrl.dim.x);
    
%     par.attCtrl.Ts = 0.01;

    % prediction matrix
    [T,S] = predmodgen(LTI, par.angCtrl.dim);
    
    % cost function
    Qbar = blkdiag(kron(eye(par.angCtrl.dim.N), par.angCtrl.Q), par.angCtrl.P);
    Rbar = kron(eye(par.angCtrl.dim.N), par.angCtrl.R);

    uref = uref(:);
    xref = xref(:);
    
    v_lim = par.cstr.maxVel;
    a_lim = par.cstr.maxAcc;
    
    H = S'*Qbar*S + Rbar;
    h = Rbar'*uref + S'*Qbar*T*(att) + S'*Qbar*xref;
    
%% Optimization

cvx_begin 
    variable u_opt(par.angCtrl.dim.u*par.angCtrl.dim.N)
    minimize(0.5*u_opt'*H*u_opt+h'*u_opt)
%     subject to
%     u_opt <= v_lim *ones(4*N,1);
cvx_end

[u] = u_opt(1:par.angCtrl.dim.u);
%% 
%     sysd = c2d(sysc,par.attCtrl.Ts);
%     sim_vec = par.dim.N/par.env.Ts;   % horizon/sampling time
% 
%     F_x_ang = kron(Q, eye(par.attCtrl.dim.N));
%     F_u_ang = kron(R, eye(par.attCtrl.dim.N));
%     F_N = P;
    
%     x_ang = zeros(par.dim.x_ang,sim_vec);    % state trajectory
%     u_ang = zeros(par.dim.u_ang,sim_vec);    % control inputs
%     y_ang = zeros(par.dim.x_ang,sim_vec);    % measurements 

%     % V(u_N) = Sum 1/2[ x(k)'Qx(k) + u(k)'Ru(k) ] + x(N)'Px(N)
end
