function [y2] = GL4(fcn, y1, par)
    %% GAUSS-LEGENDRE METHOD FOR ODE INTEGRATION
    % Fourth-order implicit integration method (assuming time invariant
    % fcn
    % From MPC book [Rawlings, p510]

    A = [0.25, 0.25-sqrt(3)/6;
         0.25+sqrt(3)/6, 0.25];
    n = numel(y1);
    
    fnc_gl = @(k) [-k(1:n) + fcn(y1 + par.sim.h*(A(1,1)*k(1:n) + A(1,2)*k(n+1:end)));
                   -k(n+1:end) + fcn(y1 + par.sim.h*(A(2,1)*k(1:n) + A(2,2)*k(n+1:end)))];
                   
    % TODO: better alternative for fsolve with the stupid option argument
    k = fsolve(fnc_gl, zeros(2*n, 1), par.settings.solve);
    
    y2 = y1 + par.sim.h*(0.5*k(1:n) + 0.5*k(n+1:end));
end

