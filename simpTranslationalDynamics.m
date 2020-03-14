function [LTI_trans] = simpTranslationalDynamics(setpt, par)
    % Linearization of the dynamics around setpoint
    %   setpt = [T_total, phi0, theta0, psi0]'
    %   x = [dx dy dz x y z]'
    %   u = [T phi_ref theta_ref]'

    T0 = setpt(1);
    phi0 = setpt(2);
    theta0 = setpt(3);
    psi0 = setpt(4);
    
    LTI_trans = struct();
   
    A = [zeros(3, 6); zeros(3) eye(3)]; 
    
    % Columns of input matrix
    bT = -1/par.drone.m*...
        [sin(phi0)*sin(psi0) + cos(phi0)*cos(psi0)*sin(theta0);
         cos(phi0)*sin(psi0)*sin(theta0) - cos(psi0)*sin(phi0);
         cos(phi0)*cos(theta0)];
     
    bphi = -T0/par.drone.m*...
           [cos(phi0)*sin(psi0) - sin(phi0)*cos(psi0)*sin(theta0);
            -sin(phi0)*sin(psi0)*sin(theta0) - cos(psi0)*cos(phi0);
            -sin(phi0)*cos(theta0)];
    
    btheta = -T0/par.drone.m*...
             [cos(phi0)*cos(psi0)*cos(theta0);
             cos(phi0)*sin(psi0)*cos(theta0);
             -cos(phi0)*sin(theta0)];
    
    B = [bT, bphi, btheta;
         zeros(par.posCtrl.dim.u)];
    C = eye(par.posCtlr.dim.y); % Assume full state feedback (for now)
    D = zeros(par.posCtlr.dim.y, par.posCtrl.dim.u);
    
    LTI_trans = ss(A, B, C, D);
end