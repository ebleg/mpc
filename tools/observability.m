function observability(LTI)

A = LTI.A;
C = LTI.C;

obsv_matrix = ctrb(A,C);
obsv_rank = rank(obsv_matrix);

st_rank = size(A);

if obsv_rank == st_rank
    disp('System is observable')
else
    disp('Not all states are observable')
end

end