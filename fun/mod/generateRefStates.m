function [ref_pos, ref_ang, ref_T] = generateRefStates(path, par)
    % Assumed the reference yaw angle is always tangent to the path
    x = path(1,:);
    y = path(2,:);
    z = path(3,:);

    ref_psi = [atan2(diff(y), diff(x)), 0]; % dimensions consistent, same reference for last and one-but-last state is acceptable
    dr = diff(path, 1, 2)/par.sim.h;
    ddr = diff(path, 2, 2)/par.sim.h/par.sim.h;
    dr = [dr dr(:,end)];
    ddr = [ddr ddr(:,end) ddr(:,end)]; % Acceleration along path
    
    ref_psi(end) = ref_psi(end-1);

    ref_pos = [dr; x; y; z];
        
    ref_ang = zeros(3,numel(x));
    ref_T = zeros(1,numel(x));
    ref_ang(3,:) = ref_psi;
    tol = 1e-2;
    settings = optimoptions(@fsolve, 'Display', 'none');
    
    % Solve for steady-state reference values
    for i=1:numel(x)
       f = @(uref) ddr(:,i) - accelerations(ref_pos(:,i), [uref; ref_psi(i)], par);
%        J_f = @(uref) simpTranslationalDynamics([uref; ref_psi(i)], par).B(1:3,:);
       
       % Solve equation with Newton-Rhapson
%        u_pos = [par.drone.m*par.env.g, 0, 0]'; 
%        while any(abs(f(u_pos)) > tol)
%            u_pos = u_pos - J_f(u_pos)\f(u_pos);
%        end
        u_pos = fsolve(f, [par.drone.m*par.env.g, 0, 0]', settings);
        ref_ang(1:2,i) = u_pos(2:3);
        ref_T(i) = u_pos(1);
    end
end

function acc = accelerations(x_pos, u_pos, par)
    tmp = translationalDynamics(x_pos, u_pos, par);
    acc = tmp(1:3);
end
    