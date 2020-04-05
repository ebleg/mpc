function [LTI_rot] = simpRotationalDynamics(par, xref)
% Generate state-space matrices for the simplified rotational dynamics of
% the quadrotor

    % State and input definition
    % xref = [phidot thetadot psidot phi theta psi]'
    % We don't need a reference for U since the dynamics are already linear
    % with respect to the input
    g1 = @(sp) rotationalDynamics(sp, [par.drone.m*par.env.g 0 0 0]', par);
    A = jacobianest(g1, xref);
    g2 = @(sp) rotationalDynamics(xref, sp, par);
    B = jacobianest(g2, [par.drone.m*par.env.g 0 0 0]');    
    B = B(:,2:end);
    C = eye(6); % Assume full state knowledge for now
    D = zeros(6, 3);
    
    LTI_rot = ss(A, B, C, D);
end

