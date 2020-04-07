function [] = plotTerminalSet(par)
    P = par.posCtrl.P;
    Xf = par.posCtrl.Xf;
    
    lim = sqrt(Xf/min(diag(P)));
    range = linspace(-lim,lim, 100);
    [X, Y, Z] = meshgrid(range, range, range);
    inTerminalSet = false(size(X));
    
    for i=1:numel(X)
        vec = [0 0 0 X(i) Y(i) Z(i)]';
        if vec'*P*vec <= Xf
           inTerminalSet(i) = true; 
        end
    end
    
    figure;
    subplot(121);
    suptitle('Terminal set visualisation')
    scatter3(X(inTerminalSet), Y(inTerminalSet), Z(inTerminalSet), '.');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    grid; grid minor;
    title('Terminal set for position')
    axis equal;
    
    for i=1:numel(X)
        vec = [X(i) Y(i) Z(i) 0 0 0]';
        if vec'*P*vec <= Xf
            inTerminalSet(i) = true;
        end
    end
   
    subplot(122);
    scatter3(X(inTerminalSet), Y(inTerminalSet), Z(inTerminalSet), '.');
    xlabel('dx/dt');
    ylabel('dy/dt');
    zlabel('dz/dt');
    grid; grid minor;
    title('Terminal set for speed')
    axis equal;
end

