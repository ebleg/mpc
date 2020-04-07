function [T,f] = posCstrMatrix(par)
    N = par.posCtrl.dim.N;
    
    vcstr = 4*par.cstr.maxVel^2*par.drone.rotor.Kf; % Speed constraint value
    Tnom = par.drone.m*par.env.g;
    
    T1 = [1 0 0; -1 0 0];

    tmp = repmat({T1},N,1);
    T = blkdiag(tmp{:});
    f = repmat([vcstr - Tnom; Tnom], N, 1);
end

