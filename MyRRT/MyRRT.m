close all;
clear, clc;

% draw the space
xmax = 1000;
ymax = 1000;
xmin = 0;
ymin = 0;
figure('Name','Space')
set(0,'DefaultFigureWindowStyle','docked')
axis([xmin xmax ymin ymax])
hold on

% Draw obstacles
xobs1 = [500 200  200 500 500]; % five points make four edges
yobs1 = [500 500  200 200 500];
plot(xobs1,yobs1)               % to plot polygon
fill(xobs1,yobs1,'c')           % to fill the polygon

% Specs
node_num = 1000;                % maximum number of nodes if goal not found
stp = 20;                       % max step size

% Initialization
qroot = [0,0];         
qgoal = [800,500];

plot(qroot(1) ,qroot(2) ,'or','MarkerSize',5,'MarkerEdgeColor','red'  ,'MarkerFaceColor',[1 0 0])
plot(qgoal(1) ,qgoal(2) ,'or','MarkerSize',5,'MarkerEdgeColor','green','MarkerFaceColor',[0 1 0])

j = 1;
nodes(j,:) = qroot;

while j < node_num-1
    
    if nodes(j,:) == qgoal;
        break
    end
    qrand = [floor(rand(1)*(xmin+(xmax-xmin))) , floor(rand(1)*(ymin+(ymax-ymin)))];
    plot(qrand(1), qrand(2), 'x', 'Color',  [0 0.4470 0.7410])
    
    % - search for the nearest node to the random node
    chkdist = [];
    for k = 1:size(nodes,1)
        tmpd = Dist(nodes(k,:),qrand);
        chkdist = [chkdist tmpd];
    end
    % - check distance between random point and nearest point and maximum
    % step size and creating new node
    [d,ind] = min(chkdist);
    
    qnear  = nodes(ind,:);
    qnew = Steer(qnear,qrand,d,stp);
    if(~CollisionDetect(qnear,qnew,xobs1,yobs1))
        j = j+1;
        nodes(j,:) = qnew;
        line([qnear(1), qnew(1)], [qnear(2), qnew(2)], 'Color', 'k', 'LineWidth', 2);
        drawnow
    end
 
end


nodes(j+1,:) = qgoal;