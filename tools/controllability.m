function controllability(LTI)

A = LTI.A;
B = LTI.B;

ctrl_matrix = ctrb(A,B);
ctrl_rank = rank(ctrl_matrix);

st_rank = size(A);

if ~(ctrl_rank == st_rank)
    warning('System is not controllable')
end

end