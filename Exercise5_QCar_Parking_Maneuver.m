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

%% EXERCISE 5: QCar - Parking-Style Manoeuvre
% A programmed approach-and-align route enters a marked parking bay.
% This is not a full parking controller with gear selection or sensor feedback.

clc; clear; close all;
qcar = loadrobot("quanserQCar");

scenario = robotScenario(UpdateRate=20,StopTime=7);
addMesh(scenario,"Plane",Size=[3.4 2.1],Color=[0.70 0.70 0.70]);
addMesh(scenario,"Box",Position=[0 0 0.05], ...
    Size=[3.0 1.6 0.1],Color=[0.20 0.20 0.20]);
for xMark = -1.15:0.35:-0.05
    addMesh(scenario,"Box",Position=[xMark 0 0.11], ...
        Size=[0.13 0.035 0.02],Color=[1 1 1]);
end

% Mark a parking bay to the side of the road using thin white boxes
addMesh(scenario,"Box",Position=[0.60 0.28 0.12], ...
    Size=[1.20 0.035 0.02],Color=[1 1 1]);
addMesh(scenario,"Box",Position=[0.60 0.78 0.12], ...
    Size=[1.20 0.035 0.02],Color=[1 1 1]);
addMesh(scenario,"Box",Position=[1.20 0.53 0.12], ...
    Size=[0.035 0.50 0.02],Color=[1 1 1]);

% Approach, turn into the bay, then align within the parking space
waypoints = [
    -1.25 0.00 0
    -0.80 0.00 0
    -0.45 0.12 0
    -0.15 0.36 0
     0.20 0.53 0
     0.65 0.53 0];
arrivalTimes = [0 1.2 2.4 3.8 5.3 7];
traj = waypointTrajectory(Waypoints=waypoints, ...
    TimeOfArrival=arrivalTimes,ReferenceFrame="ENU");

platform = robotPlatform("QCar",scenario,BaseTrajectory=traj);
updateMesh(platform,"RigidBodyTree",Object=qcar);

setup(scenario);
figure("Color","w");
ax = show3D(scenario);
axis(ax,"equal"); grid(ax,"on"); view(ax,40,28);ax.Color = "none";
xlim(ax,[-1.55 1.45]); ylim(ax,[-0.8 1.15]); zlim(ax,[0 0.8]);
title(ax,"Exercise 5 - QCar Parking-Style Manoeuvre");
xlabel(ax,"East (m)"); ylabel(ax,"North (m)"); zlabel(ax,"Height (m)");

while advance(scenario)
    show3D(scenario,FastUpdate=true,Parent=ax);
    drawnow limitrate;
    pause(0.02);
end
fprintf("Exercise 5 complete: QCar followed the programmed parking-bay entry route.\n");
