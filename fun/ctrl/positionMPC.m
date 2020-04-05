function [u_pos] = positionMPC(ang, pos, t, ref, par)
    %% POSITION MPC 
    persistent k; % From MPC HW set 3 
    if isempty(k)
        k = 1;
    end
   
    persistent u_i;
    if isempty(u_i)
        u_i = zeros(par.posCtrl.dim.u, 1);
    end
    
    %% Optimize input
    if k*par.posCtrl.sampleInt <= t
        
        % Adaptive MPC
%         setpt = [par.drone.m*par.env.g; ang(4); ang(5); ang(6)];
        setpt = [par.drone.m*par.env.g; 0; 0; ang(6)];

        LTI_pos = c2d(simpTranslationalDynamics(setpt, par), ...
            par.posCtrl.predInt, ...
            'zoh');
        
        % Sample reference points
        xrefSampled = interp1(ref.t, ref.x.pos', t + (0:par.posCtrl.dim.N)*par.posCtrl.predInt)';
        u_i = positionControl(LTI_pos, pos, xrefSampled, par) + setpt(1:3);
        k = k + 1;
    end
    
    u_pos = u_i;
end