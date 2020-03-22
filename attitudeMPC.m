function [u] = attitudeMPC(LTI, xref, uref)
%% Regulation MPC    
    [u] = attitudeControl(LTI, xref, uref);
    
%% Offset-free MPC with output feedback
% In case the states are not known and considering disturbances
    
end