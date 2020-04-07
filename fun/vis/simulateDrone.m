function [] = simulateDrone(ax, sol, par)
    nanfilt = ~isnan(sol.x.pos(1,:));
    baseTitle = ax.Title.String;
    p = plotDrone(ax, sol, 1, par);

    for i=2:4:length(nanfilt)
        if nanfilt(i)
            delete(p.bar1); delete(p.bar2) % remove old drone
            p = plotDrone(ax, sol, i, par);
            pause(par.sim.h/10);
            timeStr = string(i*par.sim.h);
            title(ax, strcat(baseTitle, " t = ", timeStr, "s")); 
        end
    end
end