function [feasible_set] = stabAttitude(u_ref,  par)
phi = 0;
theta = 0;
psi = 0;
Q = eye(par.dim.x);
R = eye(par.dim.u);
P = eye(par.dim.x);

minPhi = -pi;
maxPhi = pi;
dPhi = pi/100;
minTheta = -pi;
maxTheta = pi;
dTheta = pi/100;

feasible_set = zeros(round((maxPhi-minPhi)/dPhi), round((maxTheta-minTheta)/dTheta));

for phi = minPhi:dPhi:maxPhi
    for theta = minTheta:dTheta:maxTheta
        ang = [phi theta psi];
        LTI_trans = simpTranslationalDynamics(ang, u_ref);
        A = LTI_trans.A;
        B = LTI_trans.B;
        [X,L,K_LQR] = idare(A,B,Q,R);
        A_K = A-B*K_LQR;
        eig_A_K = eig(A_K);
        if real(eig_A_K)<0
            feasible_set(phi,theta)= 1;
        end
    end   
end
end
