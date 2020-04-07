function [F_u,f_u] = attCstrMatrix2(par)
    K_f = par.drone.rotor.Kf;
    K_m = par.drone.rotor.Km;
    max_vel = par.cstr.maxVel;
    par.cstr.maxAng = 0.5; %rad

    tmp_F_u = kron(eye(par.angCtrl.dim.u),[1; -1]);
    F_u = repmat(tmp_F_u, par.angCtrl.dim.N, par.angCtrl.dim.N);
    
    cstr23 = K_f * (max_vel)^2;
    cstr4 = 2 * K_m * (max_vel)^2;
    tmp_f = [cstr23 cstr23 cstr23 cstr23 cstr4 cstr4]';
    f_u = repmat(tmp_f, par.angCtrl.dim.N, 1);
    
%     tmp_F_x = kron(eye(par.angCtrl.dim.x),[1; -1]);

end