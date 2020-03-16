function [xref, uref, flag] = positionTargetSelector(LTI_pos, yref, par)
    %% POSITION REFERENCE SELECTOR
    % Selects optimal target for u and x based on a y from the system
    nx = par.posCtrl.dim.x; % shorthand notation
    nu = par.posCtrl.dim.u;
    ref = sdpvar(nx + nu, 1); % optimization variable contains both xref and uref
    C = [zeros(3), eye(3)]; % ONLY FOR TESTING WITH FULL-STATE FEEDBACK
    
    % First and second condition = steady-state condition
    % Third and fourth condition = stay within thrust limits
    constr = [(eye(nx) - LTI_pos.A)*ref(1:nx) - LTI_pos.B*ref((nx+1):end) == 0, ...
              C*ref(1:nx) == yref, ...
              ref((nx+1)) >= 0, ...
              ref((nx+1)) <= 4*par.cstr.maxVel^2*par.drone.rotor.Kf];
    % Since this is a steady state solution, the rate constraint imposed on
    % u is satisfied by definition.
    
    % Simple quadratic cost function
    obj = ref(1:nx)'*par.posTarSel.Q*ref(1:nx) + ... 
          ref((nx+1):end)'*par.posTarSel.R*ref((nx+1):end);

    sol = optimize(ref, obj, constr, par.opt.settings);
    if sol.problem == 0
        ref = value(ref);
        xref = ref(1:xn);
        uref = ref((xn+1):end);
        flag = 0;
    else
        warning('Position MPC optimization did not converge')
        flag = 1;
    end
end

