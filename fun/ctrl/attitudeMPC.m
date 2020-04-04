function u_att = attitudeMPC(ref, xref, par, t, att, x0, yref)
% x_ang: state vector for rotational dynamics, i.e. [p q r phi theta psi]'
% u_ang: input vector for rotational dynamics, i.e. [U2 U3 U4];

persistent k; % From MPC HW set 3
if isempty(k)
    k = 1;
end

persistent u_i;
if isempty(u_i)
    u_i = zeros(par.angCtrl.dim.u, 1);
end

if k*par.posCtrl.sampleInt <= t %TODO
    % xref = [zeros(3,length(ref.x.ang)); ref.x.ang(4:6,:)]; %[0 0 0 phi theta psi]'
    % uref = ref.u.ang;
    LTI = par.angCtrl.LTI;
    xrefSampled = interp1(ref.t, xref', t + (0:par.angCtrl.dim.N).*par.angCtrl.predInt)';
    u_i = attitudeControl(LTI, xrefSampled, par, att);
%     u_i = attitudeOutputControl(LTI_rot_d, par, x0, yref); % Output MPC
    k=k+1;
end

u_att = u_i;
   
end