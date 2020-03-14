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
    
    
    
    

end
