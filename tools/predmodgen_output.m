function [T,S]=predmodgen_output(LTI,dim)

%Prediction matrices generation
%This function computes the prediction matrices to be used in the
%optimization problem

% Prediction matrix from initial state
T=zeros(dim.x*(dim.N+1),dim.x);
for k=0:dim.N
    T(k*dim.x+1:(k+1)*dim.x,:)=LTI.A^k;
end

%Prediction matrix from input
S=zeros(dim.x*(dim.N+1),dim.u*(dim.N));
for k=1:dim.N
    for i=0:k-1
        S(k*dim.x+1:(k+1)*dim.x,i*dim.u+1:(i+1)*dim.u)=LTI.A^(k-1-i)*LTI.B;
    end
end

