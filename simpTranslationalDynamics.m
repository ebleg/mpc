function [LTI_trans] = simpTranslationalDynamics(ang, u_ref)
% Linearization of the dynamics around setpoint x_ref and setpoint u_ref
%     ang_ref = [phi theta psi]'

    LTI_trans = struct();
    
    R = eul2rotm(ang, 'XYZ');
    fcn = @(u) R*[0 0 u_ref]' + [0 0 -mg];
    
    A = [zeros(3, 6); zeros(3) eye(3)];
    B = jacobianest(fcn, u_ref);
    C = eye(6); % Assume full state feedback
    D = zeros(6, 1);
    
    LTI_trans = ss(A, B, C, D);
end