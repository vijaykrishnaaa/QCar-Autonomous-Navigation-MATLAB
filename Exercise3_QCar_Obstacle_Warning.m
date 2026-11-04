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

%% EXERCISE 3: QCar with Simulated LiDAR Obstacle Detection
% Loads a predefined Quanser QCar robot.
% Simulates a 2D LiDAR scan of a known rectangular obstacle.
% Uses the nearest simulated LiDAR return for safety-state decisions.
% The route is still prescribed; this is not a real physical sensor.

clc;
clear;
close all;

%% 1. Load the predefined robot
qcar = loadrobot("quanserQCar");

%% 2. Simulation settings
updateRate = 20;
arrivalTimes = [0 1.5 3 4.5 6];

waypoints = [
    -1.10  0  0
    -0.80  0  0
    -0.50  0  0
    -0.20  0  0
     0.00  0  0
];

%% 3. Obstacle and LiDAR settings
obstacleX = 0.65;
obstacleY = 0;

obstacleLength = 0.20;
obstacleWidth  = 0.30;
obstacleHeight = 0.36;

% Simulated LiDAR position relative to QCar centre
sensorOffset = 0.15;

% LiDAR scans from -60 to +60 degrees
angles = deg2rad(-60:2:60);
maxRange = 3.0;

% Safety thresholds measured from the LiDAR
warningDistance = 1.25;
emergencyDistance = 0.90;
stopDistance = 0.70;

%% 4. Create the road scenario
scenario = robotScenario( ...
    UpdateRate=updateRate, ...
    StopTime=6);

addMesh(scenario,"Plane", ...
    Size=[3.0 2.0], ...
    Color=[0.70 0.70 0.70]);

addMesh(scenario,"Box", ...
    Position=[0 0 0.05], ...
    Size=[2.7 0.9 0.1], ...
    Color=[0.20 0.20 0.20]);

% White lane markings
for xMark = -1.0:0.30:1.0
    addMesh(scenario,"Box", ...
        Position=[xMark 0 0.11], ...
        Size=[0.12 0.035 0.02], ...
        Color=[1 1 1]);
end

% Red obstacle
addMesh(scenario,"Box", ...
    Position=[obstacleX obstacleY obstacleHeight/2], ...
    Size=[obstacleLength obstacleWidth obstacleHeight], ...
    Color=[0.90 0.15 0.10]);

%% 5. Create the QCar trajectory
traj = waypointTrajectory( ...
    Waypoints=waypoints, ...
    TimeOfArrival=arrivalTimes, ...
    ReferenceFrame="ENU");

platform = robotPlatform( ...
    "QCar",scenario, ...
    BaseTrajectory=traj);

updateMesh(platform, ...
    "RigidBodyTree", ...
    Object=qcar);

%% 6. Display the 3D scene
setup(scenario);

figure("Color","w");
ax = show3D(scenario);

axis(ax,"equal");
grid(ax,"on");
view(ax,40,28);
ax.Color = "none";

xlim(ax,[-1.35 1.25]);
ylim(ax,[-1.0 1.0]);
zlim(ax,[0 0.8]);

title(ax,"QCar with Simulated LiDAR Obstacle Detection");
xlabel(ax,"East (m)");
ylabel(ax,"North (m)");
zlabel(ax,"Height (m)");

%% 7. Display a separate LiDAR scan
figure("Color","w");

polarAx = polaraxes;
scanLine = polarplot(polarAx,angles, ...
    maxRange*ones(size(angles)), ...
    "b-","LineWidth",1.5);

title(polarAx,"Simulated 2D LiDAR Scan");
rlim(polarAx,[0 maxRange]);

%% 8. Run the simulation and process LiDAR returns
lastState = "";
frame = 0;

while advance(scenario)

    frame = frame + 1;

    % Approximate time represented by this frame
    t = min((frame-1)/updateRate,arrivalTimes(end));

    % Position on the programmed straight route
    currentX = interp1( ...
        arrivalTimes,waypoints(:,1),t,"linear");

    % Simulated sensor position at the front of the QCar
    sensorX = currentX + sensorOffset;
    sensorY = 0;

    % Simulate each LiDAR ray against the obstacle rectangle
    ranges = maxRange*ones(size(angles));

    for k = 1:numel(angles)
        ranges(k) = rayRectangleRange( ...
            sensorX,sensorY,angles(k), ...
            obstacleX-obstacleLength/2, ...
            obstacleX+obstacleLength/2, ...
            obstacleY-obstacleWidth/2, ...
            obstacleY+obstacleWidth/2, ...
            maxRange);
    end

    % Nearest valid LiDAR return
    detected = ranges < maxRange;
    
    if any(detected)
        nearestDistance = min(ranges(detected));
    else
        nearestDistance = maxRange;
    end

    % Safety state based on the simulated scan
    if ~any(detected)
        state = "NO OBSTACLE DETECTED";
    elseif nearestDistance > warningDistance
        state = "SAFE";
    elseif nearestDistance > emergencyDistance
        state = "WARNING";
    elseif nearestDistance > stopDistance
        state = "EMERGENCY BRAKING";
    else
        state = "STOP REGION REACHED";
    end

    % Print only when the state changes
    if ~strcmp(state,lastState)
        fprintf("Time: %.2f s | Nearest LiDAR return: %.2f m | %s\n", ...
            t,nearestDistance,state);
        lastState = state;
    end

    % Update the LiDAR scan figure
    scanLine.RData = ranges;
    
    % Update the 3D robot scene
    show3D(scenario,FastUpdate=true,Parent=ax);

    drawnow limitrate;
    pause(0.02);
end



%% Local function: simulate a LiDAR ray against a rectangle
function distance = rayRectangleRange( ...
    sx,sy,angle,xmin,xmax,ymin,ymax,maxRange)

    dx = cos(angle);
    dy = sin(angle);

    distance = maxRange;

    % Check intersections with the two vertical edges
    if abs(dx) > 1e-10
        xEdges = [xmin xmax];

        for i = 1:2
            t = (xEdges(i)-sx)/dx;
            yHit = sy+t*dy;

            if t >= 0 && yHit >= ymin && yHit <= ymax
                distance = min(distance,t);
            end
        end
    end

    % Check intersections with the two horizontal edges
    if abs(dy) > 1e-10
        yEdges = [ymin ymax];

        for i = 1:2
            t = (yEdges(i)-sy)/dy;
            xHit = sx+t*dx;

            if t >= 0 && xHit >= xmin && xHit <= xmax
                distance = min(distance,t);
            end
        end
    end
end
