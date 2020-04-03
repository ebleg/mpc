function terminalSet(x, P)
% This function returns a control invariant set as terminal set
% X_f = {x|x'Px<=1
% Other possiblity : maximal invariant constraint admissible set for x(k+1) = A_K*x(k)

if ~(x'*P*x<=1)
    warning ('Point is not in terminal set')
end

end