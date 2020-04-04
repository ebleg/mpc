function u_att = attitudeMPC(ref, par, t, att, x0, yref)
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

if k*par.posCtrl.sampleInt <= t
    LTI_rot = par.angCtrl.LTI;
    % Regular MPC
    xrefSampled = interp1(ref.t, ref.x.ang', t + (0:par.angCtrl.dim.N).*par.angCtrl.predInt)';
    u_i = attitudeControl(LTI_rot, xrefSampled, par, att);
    % Output MPC
%     u_i = attitudeOutputControl(LTI_rot, par, x0, yref); % Output MPC
    k=k+1;
end

u_att = u_i;
   
end