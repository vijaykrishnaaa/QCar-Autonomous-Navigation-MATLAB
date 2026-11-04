%% Autonomous Navigation of a Quanser QCar in MATLAB:
%  Path Following, Obstacle Avoidance, Parking and A* Path Planning
%
% Student Name(s) and Registration Number(s):
%   Vijay Krishna    - CB.SC.U4CSE23112
%   Avinash          - CB.SC.U4CSE23116
%   Jeevan           - CB.SC.U4CSE23124
%   Manish           - CB.SC.U4CSE23125
%   Prakash          - CB.SC.U4CSE23610
%   Tejeswara Reddy  - CB.SC.U4CSE23614
% Mentor: Dr. A. Baskar
% Course: 23CSE472 – Artificial Intelligence and Robotics
% Department: Department of Computer Science and Engineering
% Institution: School of Computing, Amrita Vishwa Vidyapeetham, Coimbatore, India.

%% EXERCISE 6: QCar - A* Grid Path Planning and Route Playback
% A* computes a collision-free path through a 2-D grid. The resulting path
% is converted to world waypoints and played by the predefined QCar model.
% The route is planned in the grid; this is not live sensor-based replanning.

clc; clear; close all;
qcar = loadrobot("quanserQCar");

%% Build occupancy grid (1 = blocked, 0 = free)
numRows = 11;
numCols = 25;
map = false(numRows,numCols);
map(5:7,12:15) = true;    % rectangular obstacle in the middle
startRC = [6 2];
goalRC  = [6 24];

[pathRC,found] = simpleAStar(map,startRC,goalRC);
if ~found
    error("A* could not find a path. Check the grid and obstacle locations.");
end

% Convert grid cells to world positions in metres
xWorld = -1.2 + (pathRC(:,2)-1)*0.1;
yWorld = -0.5 + (pathRC(:,1)-1)*0.1;
waypoints = [xWorld yWorld zeros(size(xWorld))];

% Time each point by distance; keep times strictly increasing
stepDistances = sqrt(sum(diff(waypoints(:,1:2),1,1).^2,2));
travelSpeed = 0.25; % m/s, nominal playback speed
arrivalTimes = [0; cumsum(stepDistances)/travelSpeed];
stopTime = arrivalTimes(end);

%% Create matching 3-D scenario
scenario = robotScenario(UpdateRate=20,StopTime=stopTime);
addMesh(scenario,"Plane",Size=[3.3 1.8],Color=[0.70 0.70 0.70]);
addMesh(scenario,"Box",Position=[0 0 0.05], ...
    Size=[2.8 1.4 0.1],Color=[0.20 0.20 0.20]);

% Lane edges and centre dashes
addMesh(scenario,"Box",Position=[0 0.62 0.11], ...
    Size=[2.8 0.025 0.02],Color=[1 1 1]);
addMesh(scenario,"Box",Position=[0 -0.62 0.11], ...
    Size=[2.8 0.025 0.02],Color=[1 1 1]);
for xMark = -1.1:0.35:1.1
    addMesh(scenario,"Box",Position=[xMark 0 0.11], ...
        Size=[0.13 0.03 0.02],Color=[1 1 1]);
end
% Visible obstacle matching grid block approximately
addMesh(scenario,"Box",Position=[0.05 0 0.18], ...
    Size=[0.40 0.30 0.36],Color=[0.90 0.15 0.10]);

traj = waypointTrajectory(Waypoints=waypoints, ...
    TimeOfArrival=arrivalTimes,ReferenceFrame="ENU");
platform = robotPlatform("QCar",scenario,BaseTrajectory=traj);
updateMesh(platform,"RigidBodyTree",Object=qcar);

setup(scenario);
figure("Color","w");
ax = show3D(scenario);
axis(ax,"equal"); grid(ax,"on"); view(ax,40,28);ax.Color = "none";
xlim(ax,[-1.45 1.45]); ylim(ax,[-0.85 0.85]); zlim(ax,[0 0.8]);
title(ax,"Exercise 6 - QCar Following A* Planned Route");
xlabel(ax,"East (m)"); ylabel(ax,"North (m)"); zlabel(ax,"Height (m)");
hold(ax,"on");
plot3(ax,waypoints(:,1),waypoints(:,2),0.13*ones(size(waypoints,1),1), ...
    "c--","LineWidth",1.2);

% Also show A* result in a separate 2-D grid figure
figure("Color","w");
imagesc(~map);
colormap(gray(2)); axis equal tight; set(gca,"YDir","normal");
hold on;
plot(pathRC(:,2),pathRC(:,1),"b-","LineWidth",2);
plot(startRC(2),startRC(1),"go","MarkerFaceColor","g","MarkerSize",8);
plot(goalRC(2),goalRC(1),"ro","MarkerFaceColor","r","MarkerSize",8);
title("A* Grid Path: Green = Start, Red = Goal, Blue = Planned Path");
xlabel("Grid column"); ylabel("Grid row");

while advance(scenario)
    show3D(scenario,FastUpdate=true,Parent=ax);
    drawnow limitrate;
    pause(0.02);
end
fprintf("Exercise 6 complete: A* found a path with %d grid points.\n",size(pathRC,1));
fprintf("The QCar then followed those planned waypoints.\n");

%% Local function: simple 4-neighbour A* search
function [path,found] = simpleAStar(blocked,start,goal)
    [nr,nc] = size(blocked);
    g = inf(nr,nc);
    f = inf(nr,nc);
    closed = false(nr,nc);
    parentR = zeros(nr,nc);
    parentC = zeros(nr,nc);
    open = start;
    g(start(1),start(2)) = 0;
    f(start(1),start(2)) = heuristic(start,goal);
    moves = [-1 0; 1 0; 0 -1; 0 1];
    found = false;

    while ~isempty(open)
        vals = zeros(size(open,1),1);
        for ii = 1:size(open,1)
            vals(ii) = f(open(ii,1),open(ii,2));
        end
        [~,best] = min(vals);
        cur = open(best,:);
        open(best,:) = [];
        r = cur(1); c = cur(2);
        if closed(r,c), continue; end
        closed(r,c) = true;
        if isequal(cur,goal)
            found = true;
            break;
        end
        for jj = 1:size(moves,1)
            rr = r + moves(jj,1);
            cc = c + moves(jj,2);
            if rr < 1 || rr > nr || cc < 1 || cc > nc
                continue;
            end
            if blocked(rr,cc) || closed(rr,cc)
                continue;
            end
            tentative = g(r,c) + 1;
            if tentative < g(rr,cc)
                parentR(rr,cc) = r;
                parentC(rr,cc) = c;
                g(rr,cc) = tentative;
                f(rr,cc) = tentative + heuristic([rr cc],goal);
                if ~ismember([rr cc],open,"rows")
                    open(end+1,:) = [rr cc]; %#ok<AGROW>
                end
            end
        end
    end

    path = [];
    if ~found, return; end
    cur = goal;
    path = cur;
    while ~isequal(cur,start)
        r = cur(1); c = cur(2);
        prev = [parentR(r,c),parentC(r,c)];
        if all(prev == 0)
            found = false; path = []; return;
        end
        path = [prev; path]; %#ok<AGROW>
        cur = prev;
    end
end

function h = heuristic(a,b)
    h = abs(a(1)-b(1)) + abs(a(2)-b(2));
end
