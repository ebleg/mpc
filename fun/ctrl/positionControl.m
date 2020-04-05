function [u] = positionControl(LTI_pos, pos, xref, par)
    %% POSITION CONTROLLER
    % u:   [Ttotal, phi_ref, theta_ref]'
    % ang: current attitude of the quadcopter (which makes this MPC
    %      controller adaptive)
    % uref, xref: reference inputs and states from positionTargetSelector
    % par: parameter struct containing physical parameters, design
    %      variables etc.

    %% Define optimization problem
    % Prediction matrices
    [T, S] = predmodgen(LTI_pos, par.posCtrl.dim);
    % Extended weights
    Qbar = blkdiag(kron(eye(par.posCtrl.dim.N), par.posCtrl.Q), par.posCtrl.P);
    Rbar = kron(eye(par.posCtrl.dim.N), par.posCtrl.R);
    % Stack references vertically
    xref = reshape(xref, [(par.posCtrl.dim.N+1)*par.posCtrl.dim.x, 1]);
    Tf = T((end-par.posCtrl.dim.x+1):end,:);
    Sf = S((end-par.posCtrl.dim.x+1):end,:);
    xf = xref((end-par.posCtrl.dim.x+1):end);
    Psqrt = chol(par.posCtrl.P);
    
    % Define quadratic programming problem
    H = S'*Qbar*S + Rbar;
%     Hsqrt = sqrtm(H);
%     h = S'*Qbar*T*(pos) - S'*Qbar*xref;% - Rbar'*uref;
    h = pos'*T'*Qbar*S - xref'*Qbar*S;% - Rbar'*uref;
%     =X0'*Gx'*Q_hat*Gu+W'*Gw'*Q_hat*Gu-kron(ones(N,1),Xr)'*Q_hat*Gu;

    %% Solve optimization problem
    cvx_begin quiet
        variable u_N(par.posCtrl.dim.u*par.posCtrl.dim.N)
        minimize(quad_form(u_N, H) + 2*h*u_N);

        % Input constraints
        par.posCtrl.T*u_N <= par.posCtrl.f; 
        % Terminal set
        norm(Psqrt*(Tf*pos + Sf*u_N - xf)) <= par.posCtrl.Xf; % avoid use of quad_form when possible
    cvx_end
    
    u = u_N(1:par.posCtrl.dim.u);
    
end