function [u_opt] = attitudeOutputControl(LTI_rot, par, x0, yref)
%% Stability analysis
controllability(LTI_rot)
observability(LTI_rot)

%% Offset-free MPC with output feedback
% In case the states are not known and considering disturbances
dim = par.angCtrl.dim;

LTI = struct();
LTI.A = LTI_rot.A;
LTI.B = LTI_rot.B;
LTI.C = LTI_rot.C;
LTI.D = LTI_rot.D;
LTI.Cd=[0.3,0,0;0,0.1,0;0,0,0.2;0,0,0;0,2,0;0.2,0.3,0];

LTI.Bd=[2,0.1,0;0,0.4,0.7;0,0,0;0,0.3,0;0.1,0,0;0,0,0.8];

par.angCtrl.dim.y = 6;     %output dimension
par.angCtrl.dim.d = 3;     %disturbance dimension

% LTI.x0=[1,1,1,1,1,1]';
% LTI.yref=[1,1,1,1,1,1]';
LTI.x0 = x0;
LTI.d=[0.2,0.2,0.2]';
LTI.yref = yref;

% Extended system computation

LTI_e.A=[LTI.A LTI.Bd; zeros(par.angCtrl.dim.d,par.angCtrl.dim.x) eye(par.angCtrl.dim.d)];
LTI_e.B=[LTI.B; zeros(par.angCtrl.dim.d,par.angCtrl.dim.u)];
LTI_e.C=[LTI.C LTI.Cd];
LTI_e.x0=[LTI.x0; LTI.d];
LTI_e.yref=LTI.yref;

[dime.x, ~] = size(LTI_e.A);
[~, dime.u] = size(LTI_e.B);
[dime.y, ~] = size(LTI_e.C);
dime.N = par.angCtrl.dim.N;
dime.d = 3;

% Definition of quadratic cost function
Q_e=blkdiag(par.angCtrl.Q,zeros(par.angCtrl.dim.d));            %weight on output
R_e=par.angCtrl.R;                                              %weight on input
P_e=blkdiag(par.angCtrl.P,zeros(par.angCtrl.dim.d));            %terminal cost

% Offset-free MPC from output

%% Stability 
rank_aug = rank([eye(par.angCtrl.dim.x)-LTI.A -LTI.Bd; LTI.C LTI.Cd]);
rank_dist = rank(LTI.A)+rank(LTI.Bd);
disp(['Rank of augmented dynamics = ', num2str(rank_aug), ', rank of system = ', num2str(rank_dist)]);

%% Optimization
[T,S]=predmodgen_output(LTI_e,dime);

% Constraint setup
% a_lim = par.cstr.maxAcc*ones(N-1,1);
% x_lim = par.cstr.maxAng;
% 
% F = zeros(N-1,N*par.angCtrl.dim.u);
% for i = 1:(N-1)
%    F(i, (i-1)*nu + 1) = -1;
%    F(i, i*nu + 1) = 1;
% end

Qbar=blkdiag(kron(eye(dime.N),Q_e),P_e);
Rbar=kron(eye(dime.N),R_e);
H_e=S'*Qbar*S+Rbar;
hx0=S'*Qbar*T;
hxref=-S'*Qbar*kron(ones(dime.N+1,1),eye(dime.x));
huref=-Rbar*kron(ones(dime.N,1),eye(dime.u));
h_e=[hx0 hxref huref];
    
[~,~,G] = dare(LTI_e.A',LTI_e.C',eye(dime.x), eye(dime.u + dime.d));
G = G';

y = zeros(dim.x, simTime);
x_r = zeros(dime.x, simTime);
u_r = zeros(dime.u, simTime);
d_hat = zeros(dime.d, simTime);

x_e(:,1)=LTI_e.x0; 
x_e_hat(:,1)=[0; 10; 0; 0; 0; 0; 0; 2; 0];
y(:,1)=LTI_e.C*LTI_e.x0;

for k=1:par.sim.h:par.sim.tmax+1
    
    x_e_0=x_e(:,k);
    d_hat(:,k)=x_e_hat(end-dime.d+1:end,k);
    
    A_OTS = [eye(dim.x)-LTI.A LTI.B ;LTI.C zeros(dim.x, dim.u)];
    b_OTS = [LTI.Bd*d_hat(:,k); LTI.yref-LTI.Cd*d_hat(:,k)];
    
    H_OTS=blkdiag(zeros(dim.x),eye(dim.u));
    h_OTS=zeros(dim.x+dim.u,1);
    
    cvx_begin quiet
        variable x_r_u_r(dim.x+dim.u)
        minimize (1/2*x_r_u_r'*H_OTS*x_r_u_r + h_OTS'*x_r_u_r)
        subject to
        A_OTS * x_r_u_r <= b_OTS;
    cvx_end
%     x_r_u_r=quadprog(H_OTS,h_OTS,[],[],A_OTS, b_OTS,[],[],[],par.settings.opts);
    x_r = x_r_u_r(1:dim.x);
    u_r = x_r_u_r(dim.x+1:end);
    
    x_r_e = [x_r;d_hat(:,k)];
    
    cvx_begin quiet
        variable u_N(dim.u*dim.N)
        minimize ( (1/2)*quad_form(u_N,H_e) + (h_e*[x_e_0; x_r_e; u_r])'*u_N )
        subject to
        % input contraints
        par.angCtrl.F*u_N <= par.angCtrl.f;
%         F * u_N <= a_lim
%         % state constraints
%         S*u_N <= -T*att + x_lim;
%         S*u_N >= -T*att - x_lim;
    cvx_end
    
    u_opt(:,k) = u_N(1:dim.u);
    
    % Compute the state/output evolution
    x_e(:,k+1)=LTI_e.A*x_e_0 + LTI_e.B*u_opt(:,k);
    y(:,k+1)=LTI_e.C*x_e(:,k+1);
 
    % Update extended-state estimation
    x_e_hat(:,k+1)=LTI_e.A*x_e_hat(:,k)+LTI_e.B*u_opt(:,k)+G*(y(:,k)-LTI_e.C*x_e_hat(:,k));

    % Calculations on the real system
%     x(:,k+1) = LTI.A*x(:,k) + B*u(:,k) + LTI.Bd*d_hat(:,k); %
%     y(:,k) = LTI.C*x(:,k) + LTI.Cd*d_hat(:,k);

%     [P,L,G] = dare(A,B,par.angCtrl.Q,par.angCtrl.R);
%     Vf(k) = 0.5*x(:,k)'*P*x(:,k);
%     l(k) = 0.5*x(:,k)'*Q*x(:,k);
end

end