function [dx_ang] = rotationalDynamics(x_ang, u_ang, par)
    %% NONLINEAR ROTATIONAL QUADCOPTER DYNAMICS
    angRates = x_ang(1:3); % Vector [p q r]'
    F = diag([par.drone.l par.drone.l 1])*u_ang(2:4); % Forcing vector
    
    %% Determine relative rotor speed
    omega_r = [1 -1 1 -1]*sqrt(abs(par.drone.u2omega*u_ang));
    
    %% Rotational dynamics
    dx_ang = [par.drone.I\(F - cross(angRates, par.drone.I*angRates) ...
                             - cross(angRates, [0 0 omega_r*par.drone.rotor.I]'));
              angRates];

end