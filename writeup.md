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

