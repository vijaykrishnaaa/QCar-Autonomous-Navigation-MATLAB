# Autonomous Navigation of a Quanser QCar in MATLAB
### Path Following, Obstacle Avoidance, Parking and A* Path Planning

Six MATLAB simulations of the predefined **Quanser QCar** robot model (`loadrobot("quanserQCar")`) driving in a 3-D `robotScenario`: from simple straight-line motion up to an A* planned route around an obstacle.

## Academic details and acknowledgement

| | |
|---|---|
| **Student Name(s) and Registration Number(s)** | Vijay Krishna - CB.SC.U4CSE23112<br>Avinash - CB.SC.U4CSE23116<br>Jeevan - CB.SC.U4CSE23124<br>Manish - CB.SC.U4CSE23125<br>Prakash - CB.SC.U4CSE23610<br>Tejeswara Reddy - CB.SC.U4CSE23614 |
| **Mentor** | Dr. A. Baskar |
| **Course** | 23CSE472 – Artificial Intelligence and Robotics |
| **Department** | Department of Computer Science and Engineering |
| **Institution** | School of Computing, Amrita Vishwa Vidyapeetham, Coimbatore, India. |

This case study was carried out under the academic guidance of Dr. A. Baskar at the institution named above.

## Exercises and demo videos

| # | Script | What it shows | Video |
|---|---|---|---|
| 1 | `Exercise1_QCar_Straight_Movement.m` | QCar drives a straight route along a road | [Exercise1_Straight.mp4](videos/Exercise1_Straight.mp4) |
| 2 | `Exercise2_QCar_Curved_Path_Following.m` | QCar follows a curved set of waypoints | [Exercise2_Curved.mp4](videos/Exercise2_Curved.mp4) |
| 3 | `Exercise3_QCar_Obstacle_Warning.m` | Simulated 2-D LiDAR scan detects an obstacle; states SAFE → WARNING → EMERGENCY BRAKING → STOP | [Exercise3_LiDAR_Warning.mp4](videos/Exercise3_LiDAR_Warning.mp4) |
| 4 | `Exercise4_QCar_Obstacle_Avoidance.m` | QCar bypasses a static obstacle and returns to the lane | [Exercise4_Obstacle_Avoidance.mp4](videos/Exercise4_Obstacle_Avoidance.mp4) |
| 5 | `Exercise5_QCar_Parking_Maneuver.m` | QCar turns into and aligns inside a marked parking bay | [Exercise5_Parking.mp4](videos/Exercise5_Parking.mp4) |
| 6 | `Exercise6_QCar_AStar_Navigation.m` | A* search plans a collision-free grid path; QCar follows it in 3-D | [Exercise6_AStar.mp4](videos/Exercise6_AStar.mp4) |

## Requirements

- MATLAB (tested on MATLAB Online)
- Robotics System Toolbox (`loadrobot`, `robotScenario`, `robotPlatform`)
- Navigation Toolbox (`waypointTrajectory`)

## Steps to run

1. Download or clone this repository:
   ```
   git clone https://github.com/vijaykrishnaaa/QCar-Autonomous-Navigation-MATLAB.git
   ```
   (or click **Code → Download ZIP** and extract it).
2. Open MATLAB (desktop or [MATLAB Online](https://matlab.mathworks.com)) and upload/open the folder.
3. Open any `ExerciseN_*.m` file.
4. Click **Run** (or press **F5**). The 3-D figure opens and the QCar animation plays.
5. Read the result in the Command Window (e.g. `Exercise 1 complete: ...`).
   - Exercise 3 also opens a **LiDAR scan** figure and prints the safety state changes.
   - Exercise 6 also opens an **A\* grid** figure with the planned path.

Each script is self-contained; run them in any order.

## Notes and limitations

- Routes in Exercises 1–5 are prescribed waypoint trajectories, not closed-loop controllers.
- The LiDAR in Exercise 3 is simulated (ray–rectangle intersection), not a physical sensor.
- A* in Exercise 6 plans once on a static 11×25 occupancy grid; there is no live re-planning.
