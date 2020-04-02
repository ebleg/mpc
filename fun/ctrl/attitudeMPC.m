function u_att = attitudeMPC(ref, par, t, att, x0, yref)

persistent k; % From MPC HW set 3
if isempty(k)
    k = 1;
end

persistent u_i;
if isempty(u_i)
    u_i = zeros(par.angCtrl.dim.u, 1);
end

if k*par.angCtrl.sampleInt <= t
    xref = ref.x.ang;
    uref = ref.u.ang;
    [LTI_rot_c] = simpRotationalDynamics(par, xref);
    [LTI_rot_d] = c2d(LTI_rot_c, par.angCtrl.sampleInt, 'zoh');

    xrefSampled = interp1(ref.t, ref.x.ang', t + (0:par.angCtrl.dim.N)*par.angCtrl.predInt)';
    u_i = attitudeControl(LTI_rot_d, xref, uref, par, att, x0);
%     u_i = attitudeOutputControl(LTI_rot_d, par, x0, yref); % Output MPC
    k=k+1;
end

u_att = u_i;
   
end