function [p] = plotDrone(ax, sol, i, par)
    p = struct();
    dronePos = sol.x.pos(4:6,i);
    droneAtt = sol.x.ang(4:6,i);
    rotorPts = [1 0 0; 0 1 0; -1 0 0; 0 -1 0]'.*par.drone.l; 
    R = eul2rotm(fliplr(droneAtt'), 'ZYX');
    droneZaxis = R*[0 0 1]';
    for k=1:4
        rotorPts(:,k) = R*rotorPts(:,k) + dronePos;        
    end
    
    % For individual thrust if angular control is included as well
%     Tvec = par.drone.u2omega*[sol.u.pos(1,i); sol.u.ang(:,i)];
%     Tvec = repmat(Tvec', 3, 1);
%     droneZaxis = repmat(droneZaxis, 1, 4);
%     Tvec = Tvec.*droneZaxis;
    
    p.bar1 = line(ax, rotorPts(1,1:2:3), rotorPts(2,1:2:3), rotorPts(3,1:2:3),...
                  'Marker', 'o', 'MarkerFaceColor', 'black', ...
                  'MarkerEdgeColor', 'black', 'Color', 'black');
    p.bar2 = line(ax, rotorPts(1,2:2:4), rotorPts(2,2:2:4), rotorPts(3,2:2:4),...
                  'Marker', 'o', 'MarkerFaceColor', 'black', ...
                  'MarkerEdgeColor', 'black', 'Color', 'black');
%     p.vectors = quiver3(ax, rotorPts(1,:), rotorPts(2,:), rotorPts(3,:), ...
%                         Tvec(1,:), Tvec(2,:), Tvec(3,:));                       
end

