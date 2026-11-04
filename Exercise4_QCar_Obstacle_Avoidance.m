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

%% EXERCISE 4: QCar - Programmed Obstacle-Avoidance Manoeuvre
% The route bends around a static obstacle. The bypass path is programmed
% in advance; this example does not autonomously sense or re-plan the path.

clc; clear; close all;
qcar = loadrobot("quanserQCar");

scenario = robotScenario(UpdateRate=20,StopTime=6);
addMesh(scenario,"Plane",Size=[3.2 2.2],Color=[0.70 0.70 0.70]);
addMesh(scenario,"Box",Position=[0 0 0.05], ...
    Size=[2.9 1.7 0.1],Color=[0.20 0.20 0.20]);
for xMark = -1.1:0.35:1.1
    addMesh(scenario,"Box",Position=[xMark 0 0.11], ...
        Size=[0.13 0.035 0.02],Color=[1 1 1]);
end
% Obstacle in the middle of the direct route
addMesh(scenario,"Box",Position=[0 0 0.18], ...
    Size=[0.30 0.35 0.36],Color=[0.90 0.15 0.10]);

% Programmed bypass: move to one side, pass the obstacle, return to centre
waypoints = [
    -1.10 0.00 0
    -0.65 0.00 0
    -0.35 0.50 0
     0.35 0.50 0
     0.65 0.00 0
     1.10 0.00 0];
arrivalTimes = [0 1 2 4 5 6];
traj = waypointTrajectory(Waypoints=waypoints, ...
    TimeOfArrival=arrivalTimes,ReferenceFrame="ENU");

platform = robotPlatform("QCar",scenario,BaseTrajectory=traj);
updateMesh(platform,"RigidBodyTree",Object=qcar);

setup(scenario);
figure("Color","w");
ax = show3D(scenario);
axis(ax,"equal"); grid(ax,"on"); view(ax,40,28);ax.Color = "none";
xlim(ax,[-1.4 1.4]); ylim(ax,[-1.0 1.0]); zlim(ax,[0 0.8]);
title(ax,"Exercise 4 - QCar Obstacle-Avoidance Manoeuvre");
xlabel(ax,"East (m)"); ylabel(ax,"North (m)"); zlabel(ax,"Height (m)");
hold(ax,"on");
plot3(ax,waypoints(:,1),waypoints(:,2),0.13*ones(size(waypoints,1),1), ...
    "c--","LineWidth",1.5);

while advance(scenario)
    show3D(scenario,FastUpdate=true,Parent=ax);
    drawnow limitrate;
    pause(0.02);
end
fprintf("Exercise 4 complete: QCar followed the preplanned bypass path.\n");
