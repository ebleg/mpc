function u_att = attitudeMPC(ref, par, t, att, x0, yref)
% x_ang: state vector for rotational dynamics, i.e. [phi theta psi]'
% u_ang: input vector for rotational dynamics, i.e. [U2 U3 U4];

% t = sol.t

persistent k; % From MPC HW set 3
if isempty(k)
    k = 1;
end

persistent u_i;
if isempty(u_i)
    u_i = zeros(par.angCtrl.dim.u, 1);
end

% if k*par.angCtrl.sampleInt <= t
    xref = ref.x.ang;
    % xref = [zeros(3,length(ref.x.ang)); ref.x.ang(4:6,:)]; %[0 0 0 phi theta psi]'
    uref = ref.u.ang;
    [LTI_rot_c] = simpRotationalDynamics(par, xref);
    [LTI_rot_d] = c2d(LTI_rot_c, par.angCtrl.sampleInt, 'zoh');

    % xrefSampled = interp1(ref.t, ref.x.ang', t + (0:par.angCtrl.dim.N)*par.angCtrl.predInt)';
    u_i = attitudeControl(LTI_rot_d, xref, uref, par, att);
%     u_i = attitudeOutputControl(LTI_rot_d, par, x0, yref); % Output MPC
    k=k+1;
% end

u_att = u_i;
   
end