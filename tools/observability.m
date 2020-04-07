function observability(LTI)

A = LTI.A;
C = LTI.C;

obsv_matrix = obsv(A,C);
obsv_rank = rank(obsv_matrix);

st_rank = size(A);

if ~(obsv_rank == st_rank)
    warning('System is not observable')
end

end