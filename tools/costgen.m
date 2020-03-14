function [H, h]=costgen(x0, T, S, R, Q, P, dim)

    Qbar = blkdiag(kron(eye(dim.N), Q), P); 
    
    H = S'*Qbar*S + kron(eye(dim.N), R);   
    h = S'*Qbar*T*x0;
end