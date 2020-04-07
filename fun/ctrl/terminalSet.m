function terminalSet(x, P)
% This function returns a control invariant set as terminal set
% X_f = {x|x'Px<=1
% Other possiblity : maximal invariant constraint admissible set for x(k+1) = A_K*x(k)

if ~(x'*P*x<=1)
    warning ('Point is not in terminal set')
end

% State input constraints:
dim = par.angCtrl.dim;

% admisseble set: T * attitude + S *u_N








xref = reshape(xref, [(par.posCtrl.dim.N+1)*par.posCtrl.dim.x, 1]);
Tf = T((end-par.posCtrl.dim.x+1):end,:);
Sf = S((end-par.posCtrl.dim.x+1):end,:);
xf = xref((end-par.posCtrl.dim.x+1):end);
Psqrt = chol(par.posCtrl.P);
    
norm(Psqrt*(Tf*pos + Sf*u_N - xf)) <= par.posCtrl.Xf;
    

end