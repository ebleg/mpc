function controllability(LTI)

A = LTI.A;
B = LTI.B;

ctrl_matrix = ctrb(A,B);
ctrl_rank = rank(ctrl_matrix);

st_rank = size(A);

if ctrl_rank == st_rank
    disp('System is controllable')
else
    disp('Not all states are controllable')
end

end