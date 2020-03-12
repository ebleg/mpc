function [lti_rot] = simpRotationalDynamics(par, xref)
% Generate state-space matrices for the simplified rotational dynamics of
% the quadrotor

    % State and input definition
    % xref = [phidot thetadot psidot phi theta psi]'
    % We don't need a reference for U since the dynamics are already linear
    % with respect to the input
    
    
    lti_rot = struct();
    
    % Build up the matrices out of several parts for clarity
    A1 = [0                     -par.drone.a1*xref(3)  par.drone.a1*xref(2);
          par.drone.a3*xref(3)  0                      par.drone.a3*xref(1);
          par.drone.a5*xref(2)  par.drone.a5*xref(1)   0                   ];
    A2 = zeros(3,3);
    A3 = eye(3);
    A4 = A2;
    
    lti_rot.A = [A1 A2;
                 A3 A4]; 
    
    lti_rot.B = [diag([par.drone.b1 par.drone.b2 par.drone.b3]);
                 zeros(3,3)];
             
    lti_rot.C = eye(6); % Assume full state knowledge for now
    lti_rot.D = zeros(6, 3);
end

