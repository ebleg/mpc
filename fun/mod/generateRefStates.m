function [ref_pos, ref_ang, ref_u] = generateRefStates(path, par)
    % Assumed the reference yaw angle is always tangent to the path
    x = path(1,:);
    y = path(2,:);
    z = path(3,:);
    
    % For the reference yaw, it is assumed the drone always wants to be 
    % aligned with the tangent of the reference path

    %% Compute path time derivatives (first and second)
    ref_psi = [atan2(diff(y), diff(x)), 0]; % dimensions consistent, same reference for last and one-but-last state is acceptable
    dr = diff(path, 1, 2)/par.sim.h;
    ddr = diff(path, 2, 2)/par.sim.h/par.sim.h;
    dr = [dr dr(:,end)];
    ddr = [ddr ddr(:,end) ddr(:,end)]; % Acceleration along path
    
    
    ref_psi(end) = ref_psi(end-1);

    ref_pos = [dr; x; y; z];
        
    ref_ang = zeros(3,numel(x));
    ref_u = zeros(4,numel(x));
    ref_ang(3,:) = ref_psi;
    tol = 1e-2;
    settings = optimoptions(@fsolve, 'Display', 'none');
    
    % Solve for steady-state reference values
    for i=1:numel(x)
        f = @(u_pos) ddr(:,i) - accelerations(ref_pos(:,i), [u_pos; ref_psi(i)], par);
        u_pos = fsolve(f, [par.drone.m*par.env.g, 0, 0]', settings);
        ref_ang(1:2,i) = u_pos(2:3);
        ref_u(1,i) = u_pos(1);
    end
    
    dang = diff(ref_ang, 1, 2);
    ddang = diff(ref_ang, 2, 2);
    dang = [dang dang(:,end)];
    ddang = [ddang ddang(:,end) ddang(:,end)];
    
    ref_ang = [dang; ref_ang]; % Include rotational velocities
    for i=1:numel(x)
        g = @(u234) ddang(:,i) - angularAcc(ref_ang(:,i), [ref_u(4,i); u234], par);
        u_ang = fsolve(g, [0 0 0]', settings);
        ref_u(2:4,i) = u_ang;
    end
end

function acc = accelerations(x_pos, u_pos, par)
    tmp = translationalDynamics(x_pos, u_pos, par);
    acc = tmp(1:3);
end

function angAcc = angularAcc(x_ang, u_ang, par)
    tmp = rotationalDynamics(x_ang, u_ang, par);
    angAcc = tmp(1:3);
end
    