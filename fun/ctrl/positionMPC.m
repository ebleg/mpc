function [u_pos] = positionMPC(ang, pos, yref, par)
    %% POSITION MPC
    % Current version is for testing and assumes full state knowledge but
    % not a full state reference trajectory, i.e. it needs the controller 
    % and target selector but not the observer.
    
    setpt = [par.drone.m*par.env.g; ang];
    LTI_pos = c2d(simpTranslationalDynamics(setpt, par), ...
        par.posCtrl.Ts, ...
        'zoh');
    
    [xref, uref, flag1] = positionTargetSelector(LTI_pos, yref, par);
    [u_pos, flag2] = positionControl(LTI_pos, pos, uref, xref, par);
    
    if any([flag1, flag2])
        warning('Nonzero optimization flag')
    end
end

