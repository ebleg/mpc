function [u] = attitudeMPC(LTI, xref, uref, par, att)
%% Regulation MPC    
    
[u] = attitudeControl(LTI, xref, uref, par, att);
    
%% Offset-free MPC with output feedback
% In case the states are not known and considering disturbances

% H=predmod.S'*Qbar*predmod.S+Rbar;   
% hx0=predmod.S'*Qbar*predmod.T;
% hxref=-predmod.S'*Qbar*kron(ones(dim.N+1,1),eye(dim.nx));
% huref=-Rbar*kron(ones(dim.N,1),eye(dim.nu));
% h=[hx0 hxref huref];
    
end