clear;

run parameters

sysc = simpRotationalDynamics(par, [0 0 0 0 0 0]');
sysd = c2d(simpRotationalDynamics(par, [0 0 0 0 0 0]'), par.angCtrl.sampleInt, 'zoh');
Q = par.angCtrl.Q;
R = par.angCtrl.R;

velC = par.cstr.maxVel^2*par.drone.rotor.Kf; % Speed constraint value

%% Dare solution
[P, K, L] = idare(sysd.A, sysd.B, Q, R);
[V, D] = eig(P);

a = 4.1;
res = 20;

linspace2 = @(i) linspace(-sqrt(a/D(i,i)), sqrt(a/D(i,i)), res);
% rn = linspace(-sqrt(a/max(max(D))), sqrt(a/max(max(D))), res);

% Box grid along the principal axes of the ellipsoid
[X1 X2 X3 X4 X5 X6] = ndgrid(linspace2(1), linspace2(2), linspace2(3), linspace2(4), linspace2(5), linspace2(6));
% [X1 X2 X3 X4 X5 X6] = ndgrid(rn, rn, rn, rn, rn, rn);
Npts = numel(X1);
inTS = false(Npts, 1);

A = sysd.A;
B = sysd.B;

% LYAPUNOV DECREASE
for i = 1:Npts
    tmp = [X1(i) X2(i) X3(i) X4(i) X5(i) X6(i)]';
    x = V*tmp; % Convert to normal axes      
    if x'*P*x <= a
        if inU(-K*x, velC)
            u = -K*x;
            xnext = A*x + B*u;
            if ((0.5*xnext'*P*xnext - 0.5*x'*P*x) > -(0.5*x'*Q*x + 0.5*u'*R*u)+1e-5) % 1-e5 for numerical purposes
                disp('failed for Lyapunov decrease'), i, x'
                return;
            end
        else
            disp('failed for U'), x'
            return
        end
    end     
end

% CONTROL INVARIANCE
% for i = 1:Npts
%     tmp = [X1(i) X2(i) X3(i) X4(i) X5(i) X6(i)]';
%     x = V*tmp; % Convert to normal axes
%             
%     if x'*P*x <= a
%         inTs(i) = true;
%         if inU(-K*x, velC)
%             xnext = A*x + B*(-K*x);
%             if xnext'*P*xnext > a % Control invariance check
%                 disp('failed for invariance'), x'
%             end
%         else
%             disp('failed for U'), x'
%         end
%     end     
% end
% %%
% RNG1 = nan(Npts, 1);
% RNG2 = nan(Npts, 1);
% RNG3 = nan(Npts, 1);
% RNG4 = nan(Npts, 1);
% RNG5 = nan(Npts, 1);
% RNG6 = nan(Npts, 1);
% for i = 1:Npts
%     tmp = [X1(i) X2(i) X3(i) X4(i) X5(i) X6(i)]';
%     x = V*tmp;
%     if x'*P*x <= a
%         RNG1(i) = x(1);
%         RNG2(i) = x(2);
%         RNG3(i) = x(3);
%         RNG4(i) = x(4);
%         RNG5(i) = x(5);
%         RNG6(i) = x(6);
%     end
% end
% %%
% scatter3(RNG1, RNG2, RNG3)

function check = inU(u, velC)
    check2 = (u(1) >= -velC) && (u(1) <= velC);
    check3 = (u(2) >= -velC) && (u(2) <= velC);
    check4 = (u(3) >= -2*velC) && (u(3) <= 2*velC);
    check = check2 && check3 && check4;
end
