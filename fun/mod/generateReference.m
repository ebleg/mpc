function [ref] = generateReference(t, path, par)
    % Assumed the reference yaw angle is always tangent to the path
    fprintf('Generating reference states and inputs...')
    tic
%     r = path(t);
    r = zeros(3,numel(t));
    for i=1:numel(t)
        r(:,i) = path(t(i));
    end
    nsteps = numel(t);
    ref = struct();
    ref.t = t;

    %% Compute path time derivatives (first and second)
    % For the reference yaw, it is assumed the drone always wants to be
    % aligned with the tangent of the reference path
    dr = diff(r, 1, 2)/par.sim.h;
    psi = wrapToPi([atan2(dr(2,:), dr(1,:)), 0]); % dimensions consistent, same reference for last and one-but-last state is acceptable
    psi(end) = psi(end-1);
    
    % Compute linear velocities and accelerations along the path
    ddr = diff(r, 2, 2)/par.sim.h/par.sim.h;
    % Make lengths consistent, constant velocities and accelerations
    % assumed for last data points
    dr = [dr dr(:,end)]; % Diff operator returns array with length LEN_ORIGINAL - DEG, DEG being 1 for velocity and 2 for acceleration 
    ddr = [ddr ddr(:,end) ddr(:,end)]; 
    
    % Store results in ref container
    ref.x.pos = [dr; r];
    ref.dx.pos = [ddr; dr];
    
    ref.x.ang = zeros(par.posCtrl.dim.x, nsteps);
    ref.u.ang = zeros(par.angCtrl.dim.u, nsteps);
    ref.u.pos = zeros(par.posCtrl.dim.u, nsteps);
    ref.x.ang(6,:) = psi;
    
    tmp = [par.drone.m*par.env.g, 0, 0]';
    % Solve for steady-state reference values
    for i=1:nsteps
        f = @(u) ref.dx.pos(1:3, i) - ...
            accelerations(ref.x.pos(:,i), [u; ref.x.ang(6, i)], par);
        tmp = fsolve(f, tmp, par.settings.solve);
        ref.u.pos(:,i) = tmp;
        ref.x.ang(4:5,i) = tmp(2:3);
    end
    
    % Compute angular velocities and accelerations along the path
    dang = diff(ref.x.ang(4:6,:), 1, 2);
    ddang = diff(ref.x.ang(4:6,:), 2, 2);
    % Make array lengths consistent
    dang = [dang dang(:,end)];
    ddang = [ddang ddang(:,end) ddang(:,end)];
    ref.x.ang(1:3,:) = dang;
    ref.dx.ang = [ddang; dang];
    
    tmp = [0 0 0]';
    for i=1:nsteps
        g = @(u234) ref.dx.ang(1:3,i) - ...
            angularAcc(ref.x.ang(1:3,i), [ref.u.pos(1,i); u234], par);
        tmp = fsolve(g, tmp, par.settings.solve);
        ref.u.ang(:,i) = tmp;
    end
    fprintf(' Done - '); toc;
end

function acc = accelerations(x_pos, u_pos, par)
    tmp = translationalDynamics(x_pos, u_pos, par);
    acc = tmp(1:3);
end

function angAcc = angularAcc(x_ang, u_ang, par)
    tmp = rotationalDynamics(x_ang, u_ang, par);
    angAcc = tmp(1:3);
end
    