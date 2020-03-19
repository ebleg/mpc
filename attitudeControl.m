<<<<<<< Updated upstream
function [u] = attitudeControl(error, LTI_trans, par)
%% Attitude controller

%% Stability analysis
    LTI_pos = c2d(simpTranslationalDynamics(ang, param.drone.m*param.drone.g), ...
                  param.posCtrl.Ts, ...
                  'zoh');    
    A_dis = c2d(LTI_trans.A, par.env.Ts, 'zoh');
    B_dis = c2d(LTI_trans.B, par.env.Ts, 'zoh');

    Q = eye(par.dim.x);
    R = eye(par.dim.u);
    P = eye(par.dim.x);
    
    [X,L,K_LQR] = idare(A_dis,B_dis,Q,R);
    
    A_K = A_dis-B_dis*K_LQR;

%% Optimization problem 

    [T, S] = predmodgen(LTI_trans);
    
     % Cost function
    [J, j]=costgen(LTI_trans, T, S, R, Q, P, par.dim);
=======
function [u] = attitudeControl(att, LTI, par)
%% Attitude controller
% x_ang: state vector for rotational dynamics, i.e. [p q r phi theta psi]'
% x_pos: state vector for translational dynamics, i.e. [dx/dt dy/dt dz/dt x y z]' Likewise, two input vectors
% u_ang: input vector for rotational dynamics, i.e. [u1 u2 u3 u4]
% u_pos: input vector for translational dynamics, i.e. [u1 phi theta psi] (the attitude of the drone is considered an input to the translational dynamics)

%% Stability analysis
%     A_dis = c2d(LTI_pos.A, par.env.Ts, 'zoh');
%     B_dis = c2d(LTI_pos.B, par.env.Ts, 'zoh');
% 
%     controllability(LTI_pos)
% 
%     Q = eye(par.dim.x);
%     R = eye(par.dim.u);
%     P = eye(par.dim.x);
%     
%     [X,L,K_LQR] = idare(A_dis,B_dis,Q,R);
%     
%     A_K = A_dis-B_dis*K_LQR;

%% Regulation MPC 
    sysd = c2d(sysc,par.env.Ts);
    sim_vec = par.dim.N/par.env.Ts;   % horizon/sampling time
    
    V_f = zeros(1,sim_vec);    % terminal cost
    l = zeros(1,sim_vec);     % stage cost    
    
    Q = eye(par.dim.x_ang);
    R = eye(par.dim.u_ang);
    P = eye(par.dim.x_ang);

    F_x_ang = kron(Q, eye(par.dim.N));
    F_u_ang = kron(R, eye(par.dim.N));
    F_N = P;
    
%     x_ang = zeros(par.dim.x_ang,sim_vec);    % state trajectory
%     x_ang_0 = [0 0 0 0 0 0];
%     u_ang = zeros(par.dim.u_ang,sim_vec);    % control inputs
%     u_ang_0 = [0 0 0 0];
%     y_ang = zeros(par.dim.x_ang,sim_vec);    % measurements 

%     u_max = 0.4; % TODO: Move to parameters.m
%     f_u_ang = u_max*ones(2*par.dim.u,1);
%     f_x_ang = zeros(2*par.dim.x,sim_vec); % no constraints on the states

    % V(u_N) = Sum 1/2[ x(k)'Qx(k) + u(k)'Ru(k) ] + x(N)'Px(N)

% H = (Z'*Qbar*Z + Rbar + 2*W'*Sbar*W);
% d = (x0'*P'*Qbar*Z + 2*x0'*(A^N)'*Sbar*W)';
%     x_N = ;

H=S'*Qbar*S+kron(eye(dim.N),R);   
h=S'*Qbar*P*x0;
const=x0'*P'*Qbar*P*x0;
    
    l = x'*F_x_ang*x + u'*F_u_ang*u;
    V_f = x_N*F_N*x_N
    

%     bound_att = [f_u_ang; f_x_ang];
%     F = blkdiag(F_u_ang,F_x_ang);

% Cost function
    
%% Offset-free MPC with output feedback
% In case the states are not known and considering disturbances

>>>>>>> Stashed changes
    
    
    
    

end
