function u_att = attitudeMPC(xref, par, t, att)
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

if k*par.angCtrl.sampleInt <= t
    LTI_rot = par.angCtrl.LTI;
%     LTI_rot = par.angCtrl.LTI;
    % Regular MPC
    u_i = attitudeControl(LTI_rot, xref, par, att);
    % Output MPC
%     u_i = attitudeOutputControl(LTI, LTI_e, par, x0, yref, dime); % Output MPC
    k=k+1;
end

u_att = u_i;
   
end