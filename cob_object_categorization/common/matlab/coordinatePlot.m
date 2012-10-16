close all

A = load('../files/coordinates.txt');
%A = load('../files/coordinates_masked.txt');
B = load('../files/coordinatestf.txt');
% W = load('../files/coordinateswhole.txt');


% ab tetrapak_2_58
nicePlot = 1;

% scatter3(W(:,1), W(:,2), W(:,3), 'g.')
% hold on

% before transformation
if (0)
    figure
    if (nicePlot)
        scatter3(-A(:,3), A(:,1), -A(:,2), 'g.')
    else
        scatter3(-A(:,3), -A(:,1), -A(:,2), 'g.')
    end
    xlabel('z')
    ylabel('x')
    zlabel('y')
    axis equal
    view(55,18)
end

%after transformation
if (0)
    figure
    if (nicePlot)
        scatter3(B(:,2), -B(:,3), -B(:,1), '.', 'MarkerEdgeColor', [0.5, 1, 0.3])
    else
        scatter3(-B(:,2), -B(:,3), -B(:,1), 'g.')
    end
    hold on
    set(gca, 'FontSize', 18)
    plot3([-1,1], [-1,-1], [-1,-1], 'g', 'LineWidth', 3);
    plot3([-1,1], [-1,-1], [1,1], 'g', 'LineWidth', 3);
    plot3([-1,0], [1,1], [-1,-1], 'g', 'LineWidth', 3);
    plot3([-1,1], [1,1], [1,1], 'g', 'LineWidth', 3);
    plot3([-1,-1], [-1,1], [-1,-1], 'b', 'LineWidth', 3);
    plot3([-1,-1], [-1,1], [1,1], 'b', 'LineWidth', 3);
    plot3([1,1], [-1,1], [-1,-1], 'b', 'LineWidth', 3);
    plot3([1,1], [-1,1], [1,1], 'b', 'LineWidth', 3);
    plot3([-1,-1], [1,1], [-1,1], 'r', 'LineWidth', 3);
    plot3([-1,-1], [-1,-1], [-1,1], 'r', 'LineWidth', 3);
    plot3([1,1], [1,1], [-1,1], 'r', 'LineWidth', 3);
    plot3([1,1], [-1,-1], [-1,1], 'r', 'LineWidth', 3);
    xlabel('y')
    ylabel('z')
    zlabel('x')
    axis equal
    xlim([-1,1])
    ylim([-1,1])
    zlim([-1,1])
    view(-35,18)

    % hold on
    % plot3([0,-0.0282058]   ,   [0,0.981271] ,       [0,0.190554], 'r')
    % plot3([0,0.849774]   ,   [0,0.123925] ,       [0, -0.512374], 'g')
    % plot3([0,0.526392]   ,   [0,-0.147476] ,       [0,0.837354], 'b')
end


% after transformation
figure
hold on

% cutting planes + cuts
if (nicePlot)
    xcuts = -0.33:0.66:0.33;  % x-coordinates of cuts
    ycuts = -0.27:0.54:0.27;  % y-coordinates of cuts
else
    xcuts = -0.33:0.66:0.33;  % x-coordinates of cuts
    ycuts = -0.33:0.66:0.33;  % y-coordinates of cuts
end

for y=ycuts
    rx = [1 -1; 1 -1]*1.1;
    ry = [y y+0.001];
    rz = [1 -1]*0.70;
    surf(ry, rz, rx, 'FaceAlpha', 0.4)
end
for x=xcuts
    rx = [x x; x+0.001 x+0.001];
    ry = [-1 1]*1.00;
    rz = [-1 1]*0.70;
    surf(ry, rz, rx, [1 1; 1 1]*255, 'FaceAlpha', 0.4)
end

%coffeepot 3_35
%tetrapak 2_59..62
 
% cuts
cuts = [];
displaycutxdata = [];
displaycutx = xcuts(2);
displaycutydata = [];
displaycuty = ycuts(2);
pc = [];
distanceThreshold = 2.0/sqrt(size(B,1));
for i=1:size(B,1)
    iscut = 0;
    for x=xcuts
        if (abs(B(i, 1) - x) < distanceThreshold)
            cuts = [cuts; B(i,:)];
            iscut = 1;
            
            if (x==displaycutx)
                displaycutxdata = [displaycutxdata; B(i,:)];
            end
        end
    end
    for y=ycuts
        if (abs(B(i, 2) - y) < distanceThreshold)
            cuts = [cuts; B(i,:)];
            iscut = 1;
            
            if (y==displaycuty)
                displaycutydata = [displaycutydata; B(i,:)];
            end
        end
    end
    if (iscut==0)
        pc = [pc; B(i,:)];
    end
end
if (nicePlot)
    % cuts
    scatter3(cuts(:,2), -cuts(:,3), -cuts(:,1), 'r.')
    % point cloud
    scatter3(pc(:,2), -pc(:,3), -pc(:,1), 'g.')
else
    % cuts
    scatter3(-cuts(:,2), -cuts(:,3), -cuts(:,1), 'r.')
    % point cloud
    scatter3(-pc(:,2), -pc(:,3), -pc(:,1), 'g.')
end

axis equal
xlabel('y')
ylabel('z')
zlabel('x')
grid
xlim([-1,1])
ylim([-1,1])
zlim([-1.2,1.2])
view(-35,18)


% 2d cuts
figure
trash_axes = axes

figure
set(gca, 'FontSize', 12)

order = 2;

scatter(displaycutxdata(:,2), displaycutxdata(:,3),'.m')
hold on
for order=2:2:6
    u = displaycutxdata(:,2);
    v = displaycutxdata(:,3);
    A = zeros(size(u, 1), order+1);
    for i=1:size(u, 1)
        A(i, 1) = 1;
        for o=1:order
            A(i, o+1) = u(i)^o;
        end
    end
    x = A\v
    functionString = num2str(x(1));
    for o=1:order
        functionString = [functionString, '+', num2str(x(o+1)), '*x^', num2str(o)];
    end
    [xpl, ypl] = fplot(trash_axes, functionString, [-0.7 0.7]);
    if (order==2)
        plot(xpl, ypl, 'b', 'LineWidth', 2);
    elseif (order==4)
        plot(xpl, ypl, 'g', 'LineWidth', 2);
    else
        plot(xpl, ypl, 'r', 'LineWidth', 2);
    end
end
grid
axis equal
xlim([-0.7, 0.7])
ylim([-0.7, 0.7])
xlabel('u (former y)')
ylabel('v (former z)')
l__ = legend('Original Points', 'Order 2 Polynomial', 'Order 4 Polynomial', 'Order 6 Polynomial', 'Location', 'NorthEast')
set(l__, 'Box', 'off')
set(l__, 'FontSize', 12)


figure
set(gca, 'FontSize', 12)
scatter(displaycutydata(:,1), displaycutydata(:,3),'.')
hold on
for (order=2:2:6)
    u = displaycutydata(:,1);
    v = displaycutydata(:,3);
    A = zeros(size(u, 1), order+1);
    for i=1:size(u, 1)
        A(i, 1) = 1;
        for o=1:order
            A(i, o+1) = u(i)^o;
        end
    end
    x = A\v
    functionString = num2str(x(1));
    for o=1:order
        functionString = [functionString, '+', num2str(x(o+1)), '*x^', num2str(o)];
    end
    [xpl, ypl] = fplot(trash_axes, functionString, [-0.7 0.7]);
    if (order==2)
        plot(xpl, ypl, 'b', 'LineWidth', 2);
    elseif (order==4)
        plot(xpl, ypl, 'g', 'LineWidth', 2);
    else
        plot(xpl, ypl, 'r', 'LineWidth', 2);
    end
end
grid
axis equal
xlim([-0.7, 0.7])
ylim([-0.7, 0.7])
xlabel('u (former x)')
ylabel('v (former z)')
l__ = legend('Original Points', 'Order 2 Polynomial', 'Order 4 Polynomial', 'Order 6 Polynomial', 'Location', 'NorthEast')
set(l__, 'Box', 'off')
set(l__, 'FontSize', 12)

