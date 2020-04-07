function[zfa ]=HFalgorithm(GammaThreshold, Sinv, zhat, mFA)
% This  routine  implements the  T.J .  HO and M. Farooq algorithm  for
% generating  random points  uniformly   distributed   in   hyperellipsoid 
% Inputs :  GammaThreshold = Gating threshold (>0)7
%Sinv  =  inverse  of  covariance  matrix  S ( dim(S)=nzxnz)8
%zhat  =  center  of  the  gate  ( dim(zhat)=nzx1)9
%mFA  = number of  false   alarms  to  generate  in  the  gate10/
% Output: Zfa  = [ z  (1),...  z(mFA)] set  of  FA generated  by HF algorithm11
[V,D] = eig(Sinv );% Decomposition inv(V)Sinv
[Y,I] = sort(diag(D ));   % Sorting  of   eigenvalues  by ascending  order
A = diag(Y,0);% Diagonal matrix  of  sorted   eigenvalues
L = V(:,I );% Permutation of   eigenvectors   corresponding  to   eigenvalues
zfa = []; 
nz=size(zhat ,1);
for l = 1:mFA
    x(1) = sqrt(GammaThreshold/A(1,1))*(2*rand-1);
    for i = 2:nz
        tau_i = 0;
        for j = 1:i
            tau_i = tau_i + A(j,j)*(x(j)^2);
        end
        tau_i = GammaTreshold - tau_i;
        x(i) = sqrt(tau_i/A(i,i)*(2*rand-1);
    end
    zfa = [zfa (L*x'+ z_ha)];
end
