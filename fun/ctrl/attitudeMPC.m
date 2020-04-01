function [u] = attitudeMPC(MPC, ref, par, att, x0, yref)
xref = ref.x.ang;
uref = ref.u.ang;
[LTI_rot_c] = simpRotationalDynamics(par, xref);
[LTI_rot_d] = c2d(LTI_rot_c, par.angCtrl.sampleInt, 'zoh');

% MPC
if MPC == 0
    [u] = attitudeControl(LTI_rot_d, xref, uref, par, att);
elseif MPC == 1
    % Output MPC
    [u] = attitudeOutputControl(LTI_rot_d, par, x0, yref);
end
   
end