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
r = 60;                         % given radius around qnew

% Initialization
qroot = [0,0];         
qgoal.coord = [999,999];
qgoal.cost = 0;

plot(qroot(1) ,qroot(2) ,'or','MarkerSize',5,'MarkerEdgeColor','red'  ,'MarkerFaceColor',[1 0 0])
plot(qgoal.coord(1) ,qgoal.coord(2) ,'or','MarkerSize',5,'MarkerEdgeColor','yellow','MarkerFaceColor',[1 0.647058824 0])

j = 1;
nodes(j).coord = qroot;
% - cost is the distance from current node to start node on the path
% generated from start to that node
nodes(j).cost = 0;
nodes(j).parent = 0;
% - the first near point is the root point
% - [qnear qnew qnearest nodes] are the only structs

qnew = struct('coord',[],'cost',[],'parent',[]); % initialization

NearGoal = Dist(nodes(j).coord, qgoal.coord);

while j < node_num-1
    
    Test = Dist(nodes(j).coord, qgoal.coord);
    if Test < NearGoal
        Test1 = j;
        NearGoal = Test;
        if NearGoal <= stp
            break
        end
    end
    
    qrand = [floor(rand(1)*(xmin+(xmax-xmin))) , floor(rand(1)*(ymin+(ymax-ymin)))];
    plot(qrand(1), qrand(2), 'x', 'Color',  [0 0.4470 0.7410])

    qnearest = struct('coord',[],'cost',[],'parent',[]);

    % - search for the nearest node to the random node
    chkdist = [];
    
    for k = 1:length(nodes)
        chkdist = [chkdist Dist(nodes(k).coord,qrand)];
    end
    % - check distance between random point and near point from nodes list 
    % ,maximum step size, and create new node

    [d,ind] = min(chkdist);
    
    qnear  = nodes(ind);
    
    qnew.coord = Steer(qnear.coord,qrand,d,stp); % qnew of current iteration starts from here
    
    if(~CollisionDetect(qnear.coord,qnew.coord,xobs1,yobs1))
        % - note that we use the cost of the first near point we find to
        % get the cost of the new node
        % - that's because we are sure we will find a nearest point to new
        % point
        % - but we are not sure that we will find a point in the vicinity
        % of specific radius of new point so we can't depend on this
        % condition to get cost of new point
        
        j = j+1;
        qnew.cost = qnear.cost + Dist(qnear.coord,qnew.coord);
        qnew.parent = ind; % in case it didn't find nearer point to be its parent
        nodes(j) = qnew;
        line([qnear.coord(1), qnew.coord(1)], [qnear.coord(2), qnew.coord(2)], 'Color', 'k', 'LineWidth', 2);
        drawnow %*************************%

        Cmin = qnew.cost; %initializing Cmin
        
        % - the following loop should run after assigning new node
        % - so it should be here
        
        neghborCount = 1;
        for k = 1:length(nodes)
            if (Dist(nodes(k).coord, qnew.coord) <= r) && (~CollisionDetect(nodes(k).coord,qnew.coord,xobs1,yobs1))
                qnearest(neghborCount).coord = nodes(k).coord;
                qnearest(neghborCount).cost  = nodes(k).cost;
                if (qnearest(neghborCount).cost + Dist(qnearest(neghborCount).coord, qnew.coord) < Cmin)
                    Cmin = qnearest(neghborCount).cost + Dist(qnearest(neghborCount).coord, qnew.coord);
                    nodes(j).parent = k; % update parent of new node with the node of minimum cost
                    % - the following line has to be inside if condition
                    % the fact that if condition exist inside loop so it may be
                    % executed more than one time.
                    % - So more than one line couldexist between new point and
                    % its most nearest points at the same time
                    % - This issue will be solved in the last part of the code
                    % when choosing the best path using "parent" feature
                    line([qnearest(neghborCount).coord(1), qnew.coord(1)], [qnearest(neghborCount).coord(2), qnew.coord(2)], 'Color', 'g');
                end
                neghborCount = neghborCount + 1; % independant counter from k
            end
        end
    end
 
end


qgoal.parent = Test1;
nodes(j+1).coord = qgoal;

qend = qgoal;
while qend.parent ~= 0 %nodes(1).parent
    qstart = qend.parent;
    line([nodes(qstart).coord(1), qend.coord(1)], [nodes(qstart).coord(2), qend.coord(2)], 'Color', 'r', 'LineWidth', 2);
    qend = nodes(qstart);
end
    