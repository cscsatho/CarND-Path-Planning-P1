# **Path Planning Project**

## Project Writeup

---

**Path planning in highway environment**

The goals / steps of this project are the following:
In this project, the goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

[//]: # (Image References)

[image1]: ./img/pathplan1.jpg
[image2]: ./img/pathplan2.jpg
[image3]: ./img/pathplan3.jpg
[image4]: ./img/pathplan4.jpg

---

### Rubric points

#### 1. Compilation

_"The code compiles correctly."_

It does.

#### 2. Valid Trajectories

##### 2.1 _"The car is able to drive at least 4.32 miles without incident."_

Please see [attached video](https://github.com/cscsatho/CarND-Path-Planning-P1/blob/master/img/output4x.mp4) or launch path planner along with the simulator. Or see screenshot from below of driving 8.15 miles without any incident.

##### 2.2 _"The car drives according to the speed limit."_

A maximum of 49.5 mph was set for the car.
![image2]

##### 2.3 _"Max Acceleration and Jerk are not Exceeded."_

Maximal acceleration was set to 8.5 m/s^2 so that when lateral acceleration is added it won't exceed 10 m/s^2. Additionally, there is a 30 m space for changing lanes thus that lets us to control lateral acceleration as well. Regarding jerk, acceleration is increased/decreased gradually taking into accout previous acc/dec as well.

##### 2.4 _"Car does not have collisions."_

All three lanes are monitored and scored simultaneously by taking into account closest vehicles. This allowes us to brake when needed or change lanes when appropriate.

##### 2.5 _"The car stays in its lane, except for the time between changing lanes."_

The vehicle runs mostly on the centerline of the given lane except for the execution of the lane changing operation.

##### 2.6 _"The car is able to change lanes"_

![image4]

#### 3. Reflection - Model Documentation

##### 3.1 Telemetry data

Upon receiving a new telemetry data first of all I check wheter the car is in a lane or in the process of changing lanes.
After that I call the GetLaneScore() function.

##### 3.2 GetLaneScore

The purpose of this function is to iterate over all sensor fusion data and draw conclusions of the surrounding cars.
The function is executed for all three lanes separately. First it searches for the two closest cars: the one behind and the one in front of us in the given lane. Additionally it checks whether there is a car parallel to the ego vehicle.
A lane score is then calculated by combinig three factors: _car_s_ (distance of the given car), _car_v_ (velocity of the given car), and _car_t_ (how much time till we get to the car). All value are appropriately weighted (e.g. when a car is too close then score will become negative, or when no car is in front of us that gives a high score, etc.)

##### 3.3 Lane change decision

Based on calculated scores and other factors a decision is made whether we want ot change lanes. Only a single lanechange is allowed at once. If the neighbouring lane's score is only slightly greater but the difference is below 5% then there is no lane change executed.

##### 3.4 Adding reference points

In order to have a smooth path we are taking the last path's two ending points as start for the new curve (if available). These points are then transformed into the car's local coordinate system. Three more points are added using the car's fernet coordinates and the map's waypoints. These five points are assigned to a spline.

The remaining path points from the last trajectory are readded and the missing points that complete the series up to _STEPS_ (that is 50 points now) will be added only afterwards.

##### 3.5 Calculating additional path points

Additional points are added by getting delta_t=0.02s "slices" from the spline. I use a _maximal_acceleration_ in order to have control over the jerk and accelertion. Furthermore, the acceleration is not set instanty, but is rather incremented/decremented in in each step till it reaches the maximum/minimum. If max/min was reached in the previous etap the it will be used instead of reaching it gradually (no need for it anymore).

In the end all path points are converted back to world coordinates and sent back to the simulator.

