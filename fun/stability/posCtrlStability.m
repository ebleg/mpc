function rankChecks = posCtrlStability(par)
    fprintf('Stability of position MPC\n'); tic;
    
    %% Controllability of the linearized system
    psiref = pi;
    n = par.posCtrl.dim.x;
    res = 100;
    [PHI, THETA] = meshgrid(linspace(-pi, pi, res), linspace(-pi, pi, res));
    rankChecks = zeros(size(PHI));
    
    fprintf('\t- Checking controllability of the linearised system...'); tic;
    for i=1:numel(PHI)
        setp = [par.drone.m*par.env.g PHI(i) THETA(i) psiref]';
        rankChecks(i) = rank(ctrb(simpTranslationalDynamics(setp, par)));
    end
    fprintf(' Done - '); toc;
    if all(rankChecks)
        fprintf('\t\tAll configurations are controllable\n');
    else
        warning('System not controllable for all attitudes');
    end

    
    
end

