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

%% EXERCISE 2: Predefined Quanser QCar - Curved-Path Following
% The same predefined robot follows a gently curved set of waypoints.
% The trajectory is preprogrammed; the QCar is not detecting lane markings.

clc; clear; close all;
qcar = loadrobot("quanserQCar");

scenario = robotScenario(UpdateRate=20,StopTime=6);
addMesh(scenario,"Plane",Size=[3.2 2.0],Color=[0.70 0.70 0.70]);
addMesh(scenario,"Box",Position=[0 0 0.05], ...
    Size=[2.9 1.5 0.1],Color=[0.20 0.20 0.20]);

% Approximate road centre markings
for xMark = -1.1:0.35:1.1
    addMesh(scenario,"Box",Position=[xMark 0 0.11], ...
        Size=[0.13 0.035 0.02],Color=[1 1 1]);
end

%% Curved route: y varies to create a gentle bend
waypoints = [
    -1.05 0.00 0
    -0.70 0.00 0
    -0.35 0.22 0
     0.00 0.40 0
     0.35 0.32 0
     0.70 0.12 0
     1.05 0.00 0];
arrivalTimes = [0 1 2 3 4 5 6];
traj = waypointTrajectory(Waypoints=waypoints, ...
    TimeOfArrival=arrivalTimes,ReferenceFrame="ENU");

platform = robotPlatform("QCar",scenario,BaseTrajectory=traj);
updateMesh(platform,"RigidBodyTree",Object=qcar);

setup(scenario);
figure("Color","w");
ax = show3D(scenario);
axis(ax,"equal"); grid(ax,"on"); view(ax,40,28);
xlim(ax,[-1.35 1.35]); ylim(ax,[-1.0 1.0]); zlim(ax,[0 0.8]);
title(ax,"Exercise 2 - QCar Curved-Path Following");
xlabel(ax,"East (m)"); ylabel(ax,"North (m)"); zlabel(ax,"Height (m)");
ax.Color = "none";
while advance(scenario)
    show3D(scenario,FastUpdate=true,Parent=ax);
    drawnow limitrate;
    pause(0.02);
end
fprintf("Exercise 2 complete: QCar followed a programmed curved path.\n");
