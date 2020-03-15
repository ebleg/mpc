function [P,S,W]=predmodgen(LTI,dim)

%Prediction matrices generation
%This function computes the prediction matrices to be used in the
%optimization problem

LTI.C = eye(dim.x);
dim.y = dim.x;

%Prediction matrix from initial state
P=zeros(dim.y*(dim.N),dim.x);
for k=0:dim.N%-1
    P(k*dim.y+1:(k+1)*dim.y,:)=LTI.C*LTI.A^k;
end

%Prediction matrix from input
S=zeros(dim.y*(dim.N),dim.u*(dim.N));
for k=1:dim.N%-1
    for i=0:k-1
        S(k*dim.y+1:(k+1)*dim.y,i*dim.u+1:(i+1)*dim.u) = LTI.C*LTI.A^(k-1-i)*LTI.B;
    end
end

W = zeros(dim.y,dim.u*dim.N);
for i=0:dim.N-1
    W(1:dim.y,i*dim.u+1:(i+1)*dim.u) = LTI.C*LTI.A^(dim.N-i-1)*LTI.B; %(i+1)*ones(dim.y,dim.u);
%     W(k*dim.y+1:(k+1)*dim.y,i*dim.u+1:(i+1)*dim.u) = LTI.C*LTI.A^(k-1-i)*LTI.B;
end


