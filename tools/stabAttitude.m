% function [feasible_set] = stabAttitude(par)
% Determine the stable region for the attitude control
% xref = [phidot thetadot psidot phi theta psi]'

% phi = 0;
% theta = 0;
% psi = 0;
% Q = par.angCtrl.Q;
% R = par.angCtrl.R;
% P = par.angCtrl.P;
xref = ref.x.ang;

% minPhi = -pi;
% maxPhi = pi;
% dPhi = pi/100;
% minTheta = -pi;
% maxTheta = pi;
% dTheta = pi/100;

feasible_set = zeros(size(xref));
[rows, column] = size(feasible_set);

for i=1:rows
    for j=2:column
        LTI_rot_c = simpRotationalDynamics(par, xref);
        LTI_rot_d = c2d(LTI_rot_c, par.angCtrl.sampleInt, 'zoh');
        A_d = LTI_rot_d.A;
        B_d = LTI_rot_d.B;
        [~,~,K] = dare(A_d,B_d,Q,R);
        A_K = A_d-B_d*K;
        eig_A_K = eig(A_K);
        if real(eig_A_K(1))<0
            feasible_set(phi,theta)= 1;
        end
    end
end

% feasible_set = zeros(round((maxPhi-minPhi)/dPhi), round((maxTheta-minTheta)/dTheta));

% for phi = minPhi:dPhi:maxPhi
%     i=1;
%     for theta = minTheta:dTheta:maxTheta
%         j=1;
%         ang = [phi theta psi];
%         xref = [xref(1) xref(2) xref(3) ang];
%         LTI_rot_c = simpRotationalDynamics(par, xref);
%         LTI_rot_d = c2d(LTI_rot_c, par.angCtrl.sampleInt, 'zoh');
%         if isstable(LTI_rot_d)
%             feasible_set(i,j)= true;
%         else
%             feasible_set(i,j)= false;
%         end
%         j=j+1;
% %         A_d = LTI_rot_d.A;
% %         B_d = LTI_rot_d.B;
% %         [~,~,K] = dare(A_d,B_d,Q,R);
% %         A_K = A_d-B_d*K;
% %         eig_A_K = eig(A_K);
% %         if real(eig_A_K)<0
% %             feasible_set(phi,theta)= 1;
% %         end
%     end
%     i=i+1; 
% end
% end
