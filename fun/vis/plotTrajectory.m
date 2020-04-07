function [p] = plotTrajectory(ax, trajectory, marker, label)
    p = scatter3(ax, trajectory(4,:), ...
                  trajectory(5,:), ...
                  trajectory(6,:), marker, 'DisplayName', label);
end

