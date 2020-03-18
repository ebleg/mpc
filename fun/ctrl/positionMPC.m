function [u_pos] = positionMPC(ang, pos, uref, xref, par)
    %% POSITION MPC

%     setpt = [par.drone.m*par.env.g; ang(4:6)];
    setpt = [par.drone.m*par.env.g; 0; 0; ang(6)];

    LTI_pos = c2d(simpTranslationalDynamics(setpt, par), ...
        par.posCtrl.Ts, ...
        'zoh');
    
    u_pos = positionControl(LTI_pos, pos, uref, xref, par);
    
end

