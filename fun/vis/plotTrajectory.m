function [p] = plotTrajectory(ax, t, trajectory, marker, label)
    col = t;
    sz = ones(size(col));
    p = scatter3(ax, trajectory(4,:), ...
                  trajectory(5,:), ...
                  trajectory(6,:), sz, col, marker, 'DisplayName', label);
end

