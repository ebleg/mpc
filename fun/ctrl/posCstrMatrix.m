function [T,f] = posCstrMatrix(par)
    N = par.posCtrl.dim.N;
    nu = par.posCtrl.dim.u;
    
    nom_vel = 0.25*par.cstr.maxVel;
    vcstr = 4*par.cstr.maxVel^2*par.drone.rotor.Kf; % Speed constraint value
    acstr = 8*nom_vel*par.drone.rotor.Kf*par.posCtrl.predInt*par.cstr.maxAcc; % Rate constraint value
    Tnom = par.drone.m*par.env.g;
    
    T1 = [eye(nu); -eye(nu)];
    T2 = zeros(N-1, N*nu);
    for i = 1:(N-1)
       T2(i, (i-1)*nu + 1) = -1;
       T2(i, i*nu + 1) = 1;
    end
    tmp = repmat({T1},N,1);
    T = [blkdiag(tmp{:}); T2];
    f = [repmat([vcstr - Tnom; pi; pi; Tnom; pi; pi], N, 1); acstr*ones(N-1,1)];
end

