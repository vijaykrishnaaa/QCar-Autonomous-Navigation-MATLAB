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

%% EXERCISE 1: Predefined Quanser QCar - Straight-Line Movement
% Loads the predefined QCar model and animates it along a straight route.
% This is a prescribed trajectory, not a sensor-driven autonomous controller.

clc; clear; close all;

%% Load the predefined robot model
qcar = loadrobot("quanserQCar");

%% Create scenario
scenario = robotScenario(UpdateRate=20, StopTime=5);
addMesh(scenario,"Plane",Size=[3.0 2.0],Color=[0.70 0.70 0.70]);
addMesh(scenario,"Box",Position=[0 0 0.05], ...
    Size=[2.7 0.9 0.1],Color=[0.20 0.20 0.20]);

% Dashed center line
for xMark = -1.0:0.30:1.0
    addMesh(scenario,"Box",Position=[xMark 0 0.11], ...
        Size=[0.12 0.035 0.02],Color=[1 1 1]);
end

%% Straight route (y and z stay constant)
waypoints = [
    -0.75 0 0
    -0.45 0 0
    -0.15 0 0
     0.15 0 0
     0.45 0 0
     0.75 0 0];
arrivalTimes = [0 1 2 3 4 5];
traj = waypointTrajectory(Waypoints=waypoints, ...
    TimeOfArrival=arrivalTimes,ReferenceFrame="ENU");

%% Add the predefined QCar to the scenario
platform = robotPlatform("QCar",scenario,BaseTrajectory=traj);
updateMesh(platform,"RigidBodyTree",Object=qcar);

%% Display and animate
setup(scenario);
figure("Color","w");
ax = show3D(scenario);
axis(ax,"equal"); grid(ax,"on"); view(ax,40,28);
xlim(ax,[-1.25 1.25]); ylim(ax,[-1.0 1.0]); zlim(ax,[0 0.8]);
title(ax,"Exercise 1 - QCar Straight-Line Movement");
xlabel(ax,"East (m)"); ylabel(ax,"North (m)"); zlabel(ax,"Height (m)");
ax.Color = "none";
while advance(scenario)
    show3D(scenario,FastUpdate=true,Parent=ax);
    drawnow limitrate;
    pause(0.02);
end
fprintf("Exercise 1 complete: QCar followed a straight programmed route.\n");
