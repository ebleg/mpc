function [u] = positionControl(LTI_pos, pos, uref, xref, par)
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
%         'zoh');

    %% Define optimization problem
    % Prediction matrices
    [T, S] = predmodgen(LTI_pos, par.posCtrl.dim);
    % Extended weights
    Qbar = blkdiag(kron(eye(par.posCtrl.dim.N), par.posCtrl.Q), par.posCtrl.P);
    Rbar = kron(eye(par.posCtrl.dim.N), par.posCtrl.R);
    % Stack references vertically
    xref = reshape(xref, [(par.posCtrl.dim.N+1)*par.posCtrl.dim.x, 1]);
    uref = reshape(uref, [par.posCtrl.dim.N*par.posCtrl.dim.u, 1]);
    
    % Define quadratic programming problem
    H = S'*Qbar*S + Rbar;
    h = S'*Qbar*T*(pos) - S'*Qbar*xref - Rbar'*uref;
%     h = S'*Qbar*T*(pos - xref(1:6)) - S'*Qbar*xref - Rbar'*uref;
%     H = S'*Qbar*S+Rbar;   
%     h = S'*Qbar*T*(pos - xref(1:6));

    %% Constraint definition

    % i = 1 special case bc forward Euler instead of backward Euler for
    % rate constraint
%     vcstr = 4*par.cstr.maxVel^2*par.drone.rotor.Kf; % Speed constraint value
%     acstr = 8*par.drone.rotor.Kf*par.posCtrl.Ts*par.cstr.maxAcc; % Rate constraint value
% 
%     constr = [constr, uvec(1,1) <= vcstr, ...
%               uvec(1,1) >= 0, ... % Max rotor speed
%               uvec(2,1) - uvec(1,1) <= acstr];
% 
%     for i=2:par.posCtrl.dim.N
% %         Maximum rotor speed constraint
%         constr = [constr, uvec(i,1) <= vcstr, ...
%                   uvec(i,1) >= 0, ...;
%                   uvec(i,1) - uvec(i-1,1) <= acstr];
%     end

    %% Solve optimization problem
    cvx_begin quiet
        variable u_N(par.posCtrl.dim.u*par.posCtrl.dim.N)
        minimize ( (1/2)*quad_form(u_N,H) + h'*u_N )
%         minimize ( (1/2)*u_N'*H*u_N + d'*u_N )

%         % input constraints
%         u_N <=  u_limit*ones(4*N,1);
%         u_N >= -u_limit*ones(4*N,1);
%         % state constraints
%         Z*u_N <= -P*x0 + x_lim_vec_full;
%         Z*u_N >= -P*x0 - x_lim_vec_full; 
    cvx_end
    
    u = u_N(1:par.posCtrl.dim.u);
    
end