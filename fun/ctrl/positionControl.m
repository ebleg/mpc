function [u, flag] = positionControl(ang, uref, xref, par)
    %% POSITION CONTROLLER
    % u:   [Ttotal, phi_ref, theta_ref]'
    % ang: current attitude of the quadcopter (which makes this MPC
    %      controller adaptive)
    % uref, xref: reference inputs and states from positionTargetSelector
    % par: parameter struct containing physical parameters, design
    %      variables etc.

    %% Adaptive MPC - determine linear system matrices
    LTI_pos = c2d(simpTranslationalDynamics(ang, par.drone.m*par.drone.g), ...
                  par.posCtrl.Ts, ...
                  'zoh');

    %% Define optimization problem
    % Prediction matrices
    [T, S] = predmodgen(LTI_pos, par.posCtrl.dim);

    % Cost function
    [H, h] = costgen(x0, T, S, ...
                     par.posCtrl.R, ...
                     par.posCtrl.Q, ...
                     par.posCtrl.P, ...
                     par.posCtrl.dim);
                 
    uvec = sdpvar(N, par.posCtrl.dim.u);
    obj = uvec'*H*uvec + h'*uvec;
    
    %% Constraint definition
    constr = [];
    
    % i = 1 special case bc forward Euler instead of backward Euler for
    % rate constraint
    constr = [constr, uvec(1,1) <= 4*par.cstr.maxVel^2*par.cstr.Kt, ...
              uvec(1,1) >= 0]; % Max rotor speed
    constr = [constr, ....  % Max rotor acceleration 
              uvec(2,1) - uvec(1,1) <= 8*par.cstr.Kt*par.posCtrl.Ts*par.cstr.maxAcc];

    for i=2:N
       % Maximum rotor speed constraint
       constr = [constr, uvec(i,1) <= 4*par.cstr.maxVel^2*par.cstr.Kt, ...
                         uvec(i,1) >= 0];
       
       % Maximum rotor acceleration constraint (expressed as backward
       % Euler)
       constr = [constr, u(i,1) - u(i-1,1) <= 8*par.cstr.Kt*par.posCtrl.Ts*par.cstr.maxAcc];
    end
    
    sol = optimize(ctrs, obj, par.opt.settings);
    
    if sol.problem == 0
        uvec = value(uvec);
        u = uvec(1);
        flag = 0;
    else
        warning('Position MPC optimization did not converge')
        flag = 1;
    end
end

