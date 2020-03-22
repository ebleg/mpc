function [T,S]=predmodgen(LTI,par)

%Prediction matrices generation
%This function computes the prediction matrices to be used in the
%optimization problem

%Prediction matrix from initial state
P=zeros(par.attCtrl.dim.y*(par.attCtrl.dim.N),par.attCtrl.dim.x);
for k=0:par.attCtrl.dim.N-1
    P(k*par.attCtrl.dim.y+1:(k+1)*par.attCtrl.dim.y,:)=LTI.C*LTI.A^k;
end

%Prediction matrix from input
S=zeros(par.attCtrl.dim.y*(par.attCtrl.dim.N),par.attCtrl.dim.u*(par.attCtrl.dim.N));
for k=1:par.attCtrl.dim.N-1
    for i=0:k-1
        S(k*par.attCtrl.dim.y+1:(k+1)*par.attCtrl.dim.y,i*par.attCtrl.dim.u+1:(i+1)*par.attCtrl.dim.u)=LTI.C*LTI.A^(k-1-i)*LTI.B;
    end
end