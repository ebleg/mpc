LTI_rot = par.angCtrl.LTI;
dim = par.angCtrl.dim;

LTI = struct();
LTI.A = LTI_rot.A;
LTI.B = LTI_rot.B;
LTI.Cplot = LTI_rot.C;
LTI.C = LTI_rot.C; %LTI.C = LTI_rot.C(4:6,:);
LTI.D = LTI_rot.D;
LTI.Bd = [0.1, 0, 0.1, 0, 0, 0.1]';
% LTI.Bd = ones(6,1)
LTI.Cd = [0.5; 0.5 ; 0];
% LTI.Cd = ones(3,1)
     
LTI.x0 = [0 0 0.02 -0.5 -0.005 1.6]';
LTI.d = 0.001;

% Extended system computation
LTI_e = struct();
LTI_e.A=[LTI.A LTI.Bd; zeros(dim.d,dim.x) eye(dim.d)];
LTI_e.B=[LTI.B; zeros(dim.d,dim.u)];
LTI_e.C=[LTI.C LTI.Cd];
LTI_e.x0=[LTI.x0; LTI.d];

par.angCtrl.dime = struct();
[par.angCtrl.dime.x, ~] = size(LTI_e.A);
[~, par.angCtrl.dime.u] = size(LTI_e.B);
[par.angCtrl.dime.y, ~] = size(LTI_e.C);
par.angCtrl.dime.N = dim.N;
par.angCtrl.dime.d = 1;

pred = struct();

pred.Q_e=10*blkdiag(par.angCtrl.Q,zeros(dim.d));         %weight on output
pred.R_e=par.angCtrl.R;                                  %weight on input
pred.P_e=blkdiag(par.angCtrl.P,zeros(dim.d));            %terminal cost

pred.Qbar = blkdiag(kron(eye(dim.N),par.angCtrl.Q),par.angCtrl.P);
pred.Rbar = kron(eye(dim.N),par.angCtrl.R);
pred.Pbar = par.angCtrl.P;

[T,S]=predmodgen_output(LTI,par.angCtrl.dim);
pred.T = T;
pred.S = S;
pred.Tf = pred.T((end-par.angCtrl.dim.x+1):end,:);
pred.Sf = S((end-par.angCtrl.dim.x+1):end,:);
pred.Psqrt = chol(par.posCtrl.P);

H_e = pred.S'*pred.Qbar*pred.S+pred.Rbar;   
% h_e = [pred.S'*pred.Qbar*pred.T,...
%         -pred.S'*pred.Qbar*kron(ones(par.angCtrl.dime.N+1,1),eye(par.angCtrl.dime.x)),...
%         -pred.Rbar*kron(ones(par.angCtrl.dime.N,1),eye(par.angCtrl.dime.u))];
pred.H_e = H_e;
% pred.h_e = h_e;

G=place(LTI_e.A',LTI_e.C',[0.5; 0.4; 0.45;0.6;0.65; 0.6; 0.6])';
% [X,G,L] = idare(LTI.A',LTI.B,par.angCtrl.Q, par.angCtrl.R,[],[]);
pred.G = G; % Observer gain

%% Stability
controllability(LTI_rot)
observability(LTI_rot)

rank_aug = rank([eye(par.angCtrl.dim.x)-LTI.A -LTI.Bd; LTI.C LTI.Cd]);
rank_dist = rank(LTI.A)+rank(LTI.Bd);
if ~(rank_aug==rank_dist)
    warning('Augmented system is not observable');
end