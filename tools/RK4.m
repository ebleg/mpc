function [y2] = RK4(fcn, y1, h)
% CLASSIC RUNGE-KUTTA METHOD
    k1 = h*fcn(y1);
    k2 = h*fcn(y1 + k1/2);
    k3 = h*fcn(y1 + k2/2);
    k4 = h*fcn(y1 + k3);
    
    y2 = y1 + 1/6*(k1 + 2*k2 + 2*k3 + k4);
end

