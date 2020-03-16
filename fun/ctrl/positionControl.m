function [u, flag] = positionControl(LTI_pos, pos, uref, xref, par)
    %% POSITION CONTROLLER
    % u:   [Ttotal, phi_ref, theta_ref]'
    % ang: current attitude of the quadcopter (which makes this MPC
    %      controller adaptive)
    % uref, xref: reference inputs and states from positionTargetSelector
    % par: parameter struct containing physical parameters, design
    %      variables etc.

    %% Adaptive MPC - determine linear system matrices
%     setpt = [par.drone.m*par.env.g; ang];
%     LTI_pos = c2d(simpTranslationalDynamics(setpt, par), ...
%         par.posCtrl.Ts, ...
%         'zoh');

    %% Define optimization problem
    % Prediction matrices
    [T, S] = predmodgen(LTI_pos, par.posCtrl.dim);

    % Cost function
    Qbar = blkdiag(kron(eye(par.posCtrl.dim.N), par.posCtrl.Q), par.posCtrl.P);
    Rbar = kron(eye(par.posCtrl.dim.N), par.posCtrl.R);

    H = S'*Qbar*S + Rbar;
    h = Rbar*uref + S'*Qbar*T*pos + S'*Qbar*xref;

    uvec = sdpvar(par.posCtrl.dim.N*par.posCtrl.dim.u, 1);
    obj = 0.5*uvec'*H*uvec + h'*uvec;

    %% Constraint definition
    constr = [];

    % i = 1 special case bc forward Euler instead of backward Euler for
    % rate constraint
    vcstr = 4*par.cstr.maxVel^2*par.drone.rotor.Kf; % Speed constraint value
    acstr = 8*par.drone.rotor.Kf*par.posCtrl.Ts*par.cstr.maxAcc; % Rate constraint value

    constr = [constr, uvec(1,1) <= vcstr, ...
              uvec(1,1) >= 0, ... % Max rotor speed
              uvec(2,1) - uvec(1,1) <= acstr];

    for i=2:par.posCtrl.dim.N
        % Maximum rotor speed constraint
        constr = [constr, uvec(i,1) <= vcstr, ...
                  uvec(i,1) >= 0, ...;
                  uvec(i,1) - uvec(i-1,1) <= acstr];
    end

    %% Solve optimization problem
    sol = optimize(constr, obj, par.opt.settings);

    if sol.problem == 0
        uvec = value(uvec);
        u = uvec(1);
        flag = 0;
    else
        warning('Position MPC optimization did not converge')
        flag = 1;
    end
end

