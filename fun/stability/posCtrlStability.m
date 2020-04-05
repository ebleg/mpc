function [] = posCtrlStability(par)
    psiref = pi/4;
    n = par.posCtrl.dim.x;
    [PHI, THETA] = meshgrid(linspace(-pi, pi), linspace(-pi,pi));
    rankChecks = false(size(PHI));
    
    for i=numel(PHI)
        rankChecks(i) = ctrb(
    end
end

