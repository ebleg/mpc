function [u_out, x_0_out, xehat_0_out, e_out] = attitudeMPC(LTI, LTI_e, par, yref, pred, x_1, xehat_1, t) % xref, par, t, att
    % x_ang: state vector for rotational dynamics, i.e. [p q r phi theta psi]'
    % u_ang: input vector for rotational dynamics, i.e. [U2 U3 U4];

    persistent k; % From MPC HW set 3
    if isempty(k)
        k = 1;
    end

    persistent u_i;
    if isempty(u_i)
        u_i = zeros(par.angCtrl.dim.u, 1);
    end
    
    persistent x_0;
    if isempty(x_0)
        x_0 = zeros(par.angCtrl.dim.x, 1);
    end
    
    persistent xehat_0;
    if isempty(xehat_0)
        xehat_0 = zeros(par.angCtrl.dim.x+1, 1);
    end
  
    persistent e;
    if isempty(e)
        e = zeros(par.angCtrl.dim.x, 1);
    end
    
    if k*par.angCtrl.sampleInt <= t
        % Output MPC
        [u_i, x_0, xehat_0, e] = attitudeOutputControl(LTI, LTI_e, par, yref, pred, x_1, xehat_1);
        k = k + 1;
    end

    u_out = u_i;
    x_0_out = x_0;
    xehat_0_out = xehat_0;
    e_out = e;
end