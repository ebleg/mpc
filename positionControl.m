function [u] = positionControl(ang, error, par)
    %% POSITION CONTROLLER

    %% Adaptive MPC - determine linear system matrices
    LTI_pos = c2d(simpTranslationalDynamics(ang, par.drone.m*par.drone.g), ...
                  par.posCtrl.Ts, ...
                  'zoh');

    %% Define optimization problem
    % Prediction matrices
    [T, S] = predmodgen(LTI_pos, par.posCtrl.dim);

    % Cost function
    [H, h] = costgen(LTI, T, S, ...
                     par.posCtrl.R, ...
                     par.posCtrl.Q, ...
                     par.posCtrl.P, ...
                     par.posCtrl.dim);
    
    %% Solve optimization problem
    uvec = sdpvar(N, par.dim.u);
    % [ YALMIP MAGIC HERE ]
    
    

end

