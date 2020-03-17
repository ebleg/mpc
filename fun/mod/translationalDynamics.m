function [dx_pos] = translationalDynamics(x_pos, u_pos, par)
    %% NONLINEAR TRANSLATIONAl QUADCOPTER DYNAMICS
    linearSpeeds = x_pos(1:3);
    forcing = [0 0 u_pos(1)]'; % Total thrust
    gravity = [0 0 -par.drone.m*par.env.g]';
   
    R = eul2rotm(u_pos(2:4)', 'XYZ');
    
    dx_pos = [1/par.drone.m*(gravity + R*forcing); 
              linearSpeeds];
end