function [u] = attitudeControl(att, LTI, par)
%% Attitude controller
% x_ang: state vector for rotational dynamics, i.e. [p q r phi theta psi]'
% u_ang: input vector for rotational dynamics, i.e. [u1 u2 u3 u4]

%% Stability analysis
%     A_dis = c2d(LTI_pos.A, par.attCtrl.Ts, 'zoh');
%     B_dis = c2d(LTI_pos.B, par.attCtrl.Ts, 'zoh');
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
    par.attCtrl.dim.u = 4;
    par.attCtrl.dim.x = 6;
    par.attCtrl.dim.N = 10;
    
    par.attCtrl.Q = eye(par.attCtrl.dim.x);
    par.attCtrl.R = eye(par.attCtrl.dim.u);
    par.attCtrl.P = eye(par.attCtrl.dim.x);
    
    par.attCtrl.Ts = 0.01;

    % prediction matrix
    [T,S] = predmodgen(LTI, par.attCtrl.dim);
    
    % cost function
    Qbar = blkdiag(kron(eye(par.attCtrl.dim.N), par.attCtrl.Q), par.attCtrl.P);
    Rbar = kron(eye(par.posCtrl.dim.N), par.posCtrl.R);

    uref = uref(:);
    xref = xref(:);
    
    v_lim = ;
    a_lim = ;

    H = S'*Qbar*S + Rbar;
    h = Rbar*uref + S'*Qbar*T*att + S'*Qbar*xref;
    
%% Optimization

cvx_begin 
    variable u_opt(par.attCtrl.dim.u*par.attCtrl.dim.N)
    minimize(0.5*uopt2'*H*uopt2+h'*uopt2)
    subject to
    u_opt<=v_lim;    
cvx_end

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
