LTI_rot = par.angCtrl.LTI;
dim = par.angCtrl.dim;

LTI = struct();
LTI.A = LTI_rot.A;
LTI.B = LTI_rot.B;
LTI.Cplot = LTI_rot.C;
LTI.C = LTI_rot.C;
LTI.D = LTI_rot.D;
% LTI.Cd=zeros(6,6);
LTI.Bd = [0.1, 0  , 0  ;...
          0  , 0  , 0.1;...
          0.1, 0  , 0  ;...
          0  , 0  , 0  ;...
          0  , 0  , 0.1;...
          0  , 0.1, 0 ];
      
LTI.Cd = [0  ,0.1,0  ;...
          0  ,0  ,0.1;...
          0.1,0  ,0  ;...
          0  ,0  ,0  ;...
          1  ,0  ,0  ;...
          0  ,0  ,0 ];
     
LTI.x0 = [0 0 0.02 -0.5 -0.005 1.6]';
LTI.d = [0.01 0.01 0.01]';

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
par.angCtrl.dime.d = 3;

pred = struct();

% pred.Q_e=blkdiag(par.angCtrl.Q,zeros(dim.d));            %weight on output
% pred.R_e=par.angCtrl.R;                                  %weight on input
% pred.P_e=blkdiag(par.angCtrl.P,zeros(dim.d));            %terminal cost

pred.Qbar = blkdiag(kron(eye(dim.N),par.angCtrl.Q),par.angCtrl.P);
pred.Rbar = kron(eye(dim.N),par.angCtrl.R);
pred.Pbar = par.angCtrl.P;

% pred.Qbar=blkdiag(kron(eye(dim.N),pred.Q_e),pred.P_e);
% pred.Rbar=kron(eye(dim.N),pred.R_e);

[T,S]=predmodgen_output(LTI,par.angCtrl.dim);
pred.T = T;
pred.S = S;

H_e = pred.S'*pred.Qbar*pred.S+pred.Rbar;   
% h_e = [pred.S'*pred.Qbar*pred.T,...
%         -pred.S'*pred.Qbar*kron(ones(par.angCtrl.dime.N+1,1),eye(par.angCtrl.dime.x)),...
%         -pred.Rbar*kron(ones(par.angCtrl.dime.N,1),eye(par.angCtrl.dime.u))];
pred.H_e = H_e;
% pred.h_e = h_e;

[~,~,G] = dare(LTI_e.A',LTI_e.C',eye(par.angCtrl.dime.x), eye(par.angCtrl.dime.u + par.angCtrl.dime.d));
pred.G = G'; % Observer gain

%% Stability
controllability(LTI_rot)
observability(LTI_rot)

rank_aug = rank([eye(par.angCtrl.dim.x)-LTI.A -LTI.Bd; LTI.C LTI.Cd]);
rank_dist = rank(LTI.A)+rank(LTI.Bd);
if ~(rank_aug==rank_dist)
    warning('Augmented system is not observable');
end