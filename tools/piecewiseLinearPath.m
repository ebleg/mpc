function [r] = piecewiseLinearPath(nodes, path, par, t)
    path = path(end:-1:1);
    pathLengths = zeros(length(path),1);
    for i=2:length(path)
       pathLengths(i) = norm(nodes(:,path(i)) - nodes(:,path(i-1)));
    end
    
    totalLength = sum(pathLengths);
    tFrac = t/par.sim.tmax;
    lengthFractions = cumsum(pathLengths)./totalLength;
    ind = find(tFrac >= lengthFractions, 1, 'last');
    
    if t == par.sim.tmax
        r = nodes(:,path(end));
    else
        m = (nodes(:,path(ind+1)) - nodes(:,path(ind)))./par.sim.tmax./(lengthFractions(ind+1) - lengthFractions(ind)); 
        r = nodes(:,path(ind)) + m.*(t - par.sim.tmax.*lengthFractions(ind));
    end
end

