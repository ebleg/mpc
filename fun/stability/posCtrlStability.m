function rankChecks = posCtrlStability(par, ref, t)
    fprintf('Stability of position MPC\n'); tic;
    P = par.posCtrl.P;
    Xf = par.posCtrl.Xf;
    Q = par.posCtrl.Q;
    R = par.posCtrl.R;
    
    psiref = pi;
    n = par.posCtrl.dim.x;
    res = 10;
    [PHI, THETA] = meshgrid(linspace(-pi, pi, res), linspace(-pi, pi, res));
    rankChecks = zeros(size(PHI));

        
    for i=1:numel(PHI) % Loop over attitude mesh
        % Set up linearized system for reference attitude
        setp = [par.drone.m*par.env.g PHI(i) THETA(i) psiref]';
        
        % I presume the discretized system is used???
        sys = c2d(simpTranslationalDynamics(setp, par), par.posCtrl.predInt);
        
        % Controllability check
        rankChecks(i) = rank(ctrb(sys));
        if rankChecks(i) ~= n
            warning('System not controllable')
        end
        
        % DARE solution
        [~, K, ~] = idare(sys.A, sys.B, Q, R);
        AK = sys.A - sys.B*K;
        QK = Q + K'*R*K;       
    end
    
    limits = [];
    if ~isdiag(P)
       error('This method only works for a diagonal P') 
    end
    for i=1:n
        limits = [limits; sqrt(Xf/P(i,i))]; % ASSUMING P = DIAGONAL!
    end
        
    xref1 = interp1(ref.t, ref.x.pos', t + (0:par.posCtrl.dim.N)*par.posCtrl.predInt)';
    t2 = t + par.posCtrl.sampleInt;
    xref2 = interp1(ref.t, ref.x.pos', t2 + (0:par.posCtrl.dim.N)*par.posCtrl.predInt)';

    limitLinspace = @(i) linspace(-limits(i), limits(i), 10);
    termSetGrid = ndgrid(limitLinspace(1), limitLinspace(2), ...
                         limitLinspace(3), limitLinspace(4), ...
                         limitLinspace(5), limitLinspace(6));
    
    i = 1;                 
    while ~failed && ~done
        u = -K*termSetGrid(i);
        check1 = 
        
        if i == numel(limitLinspace)
            done = true;
        end
    end
end

