
# F1-tenth-Autonomous-Racing-Hackathon

The following content is an updated README - with some improvements to the program, courtesy of me.

✅ Increased speed (from 10m/s to 12m/s)
✅ Better PID tuning for stability at higher speeds
✅ Reduced unnecessary slowdowns during turns
✅ Adaptive speed control based on curvature

The following content is the original README of the submission, courtesy of  @harish950.

## F1Tenth_Hackathon_Submission_Team_3

![](https://github.com/harish950/F1Tenth_Hackathon_Submission_Team_3/blob/main/rviz.gif)

### Background
Our solution uses a wall following algorithm to navigate around a closed loop track. The goal was to maintain a consistent and safe distance from the wall while ensuring smooth cornering and optimal speeds. 

The grading criteria was the fastest lap time (50%) and number of consecutive laps (50%) within 10 minutes. To maximise our scores, we optimised our algorithm to achieve the maximum number of consecutive laps (with no collisons) albeit each lap being slighlty slower. 

### Algorithm Overview

#### Sensor Input
The algorithm utilizes data from the vehicle’s LaserScan (LIDAR) sensor and odometry to measure distances to obstacles and the walls

#### Control Strategy:
The algorithm uses the Proportional-Integral-Derivative (PID) Controller to maintain the desired distance from the wall. 


#### Dynamic Speed Control:
Adjusts speed dynamically based on the car’s steering angle:
- Faster on straight sections
- Slower on sharp turns to ensure stability

#### Detection of Straights:
Analyzes LaserScan data to detect straight paths by calculating the variance of distances within a certain angular range

#### Output:
- Steering angle: Adjusts the car’s direction to stay near the desired wall.
- Speed: Dynamically scaled to maximize efficiency on straights while maintaining control in turns.


### Results

The following are the results tested using the [F1 Tenth Evaluation](https://github.com/NTU-Autonomous-Racing-Team/F1Tenth_Hackathon_Evaluation/tree/main) repo

- 11 consecutive laps (no collisions)
- fastest lap of 51.9 seconds