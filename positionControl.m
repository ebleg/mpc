function [u] = positionControl(ang, error, param)
    %% POSITION CONTROLLER

    %% Adaptive MPC - determine linear system matrices
    LTI_pos = c2d(simpTranslationalDynamics(ang, param.drone.m*param.drone.g), ...
                  param.posCtrl.Ts, ...
                  'zoh');

    %% Define optimization problem
    % Prediction matrices
    [T, S] = predmodgen(LTI_pos, param.posCtrl.dim);

    % Cost function
    [H, h] = costgen(LTI, T, S, ...
                     param.posCtrl.R, ...
                     param.posCtrl.Q, ...
                     param.posCtrl.P, ...
                     param.posCtrl.dim);
    
    %% Solve optimization problem
    uvec = sdpvar(N, param.dim.u);
    % [ YALMIP MAGIC HERE ]
    
    

end

