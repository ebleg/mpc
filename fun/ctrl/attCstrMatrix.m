function [F,f] = attCstrMatrix(par)

K_f = par.drone.rotor.Kf;
K_m = par.drone.rotor.Km;
max_vel = par.cstr.maxVel;

F = zeros(2*par.angCtrl.dim.u, par.angCtrl.dim.u*par.angCtrl.dim.N);
f = zeros(2*par.angCtrl.dim.u*par.angCtrl.dim.N,1);

tmp_F = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
F = repmat(tmp_F,par.angCtrl.dim.N, 1);

cstr23 = K_f * max_vel^2;
cstr4 = 2 * K_m * max_vel^2;
tmp_f = [cstr23 cstr23 cstr23 cstr23 cstr4 cstr4]';
f = repmat(tmp_f, par.angCtrl.dim.N, 1);

end