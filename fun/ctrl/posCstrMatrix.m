function [T,f] = posCstrMatrix(par)
    N = par.posCtrl.dim.N;
    nu = par.posCtrl.dim.u;
    
    vcstr = 4*par.cstr.maxVel^2*par.drone.rotor.Kf; % Speed constraint value
    acstr = 8*par.drone.rotor.Kf*par.posCtrl.predInt*par.cstr.maxAcc; % Rate constraint value    
    T1 = [1 0 0; -1 0 0];
    T2 = zeros(N-1, N*nu);
    for i = 1:(N-1)
       T2(i, (i-1)*nu + 1) = -1;
       T2(i, i*nu + 1) = 1;
    end
    tmp = repmat({T1},N,1);
    T = [blkdiag(tmp{:}); T2];
    f = [repmat([vcstr; vcstr], N, 1); acstr*ones(N-1,1)];
end

