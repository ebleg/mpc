function [H, h]=costgen(LTI, T, S, R, Q, P, dim)

    Qbar = blkdiag(kron(eye(dim.N), Q), P); 
    
    H = S'*Qbar*S + kron(eye(dim.N), R);   
    h = S'*Qbar*T*LTI.x0;
%     const = LTI.x0'*T'*Qbar*T*LTI.x0;

end