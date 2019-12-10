# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
--
[//]: # (Image References)

[image1]: ./WriteupImages/State_Diagram.jpg "State Diagram"
[image2]: ./WriteupImages/BehaviourPlanning_BlockDiagram.jpg "Block Diagram for Behaviour Planning Module"
[image3]: ./WriteupImages/10milesCompletion.jpg "Car completes 10 miles Successfully"
[image4]: ./WriteupImages/Trajectory_Gen.jpg "Trajectory Generation using spline"
---
## Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

##### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

##### Main car's localization Data (No Noise)
["x"] The car's x position in map coordinates
["y"] The car's y position in map coordinates
["s"] The car's s position in frenet coordinates
["d"] The car's d position in frenet coordinates
["yaw"] The car's yaw angle in the map
["speed"] The car's speed in MPH

##### Previous path data given to the Planner
//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 
["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator

##### Previous path's end s and d values 
["end_path_s"] The previous list's last point's frenet s value
["end_path_d"] The previous list's last point's frenet d value

##### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)
["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Implementation Details
### Route planning
The route planning component is responsible for high-level decisions about the path of the vehicle between two points on a map; for example which roads, highways, or freeways to take to reach 'A' Location to 'B' Location. This component is similar to the route planning feature found on many smartphones or modern car navigation systems.
In our project this is already pre-defined and being implicite in the map data received.

### Prediction
The prediction component estimates what actions other objects might take in the future. For example, if another vehicle were identified, the prediction component would estimate its future Position, Velocity.
In our system we are receiving this data from Sensor Fusion Module (refer above for specific data elements and order). Using this information, we will judge the following parameters for vehicles around us lane-wise:
* 's' Distance of nearest Vehicle in Front : Based on Vehicles in Front of our Car,
* 's' Distance of nearest Vehicle in Rear : Based on Vehicles in rear of our Car,
* Each Lane's target speed : Based on speed of Vehicles moving in front of us,
* Each Lane's Traffic Density : Based on Count of Vehicles in both direction from us.

*Note* : Front Distance has been checked from our Car to 32 's' units from us. And Rear Distance has been checked from our Car to 30 's' units from us in rear direction 

### Behaviour Planning
The behavioral planning component determines what behavior the vehicle should exhibit at any point in time. This could mean stopping at a traffic light or intersection, changing lanes, accelerating, or taking a left/right turns , etc. are some of the maneuvers that may be issued by this module. Let's have a look on Behaviour Planning Module -->
![alt text][image2] 
Our project has these states :
![alt text][image1]
Let's have a look on various functions of our Behaviour Planning Module:

* Safety Check
The safe distance is considered when there is no Vehicle in the range starting from 18 's' units ahead of our car to 15 's' units behind of our car. For a vehicle in Lane '0', Lane change to Left is considered 'Not Safe' and similarly Lane change to Right from Lane '2' is considered 'Not Safe'.


* Cost Calculation
  - Cost Based on Nearest Vehicle Distance : This factor is inversely proportional to the Cost. Hence this varies [0 ~1]in case of ['s' distance is 30 ~ 's' distance is 1].
  - Cost Based on Lane Speed : This factor is inversely proportional to the Cost. Hence this varies [0 ~1]in case of [ 50 mph ~ 0 mph].
  - Cost Based on Traffic Density : This factor is directly proportional to the Cost. Hence this varies [0 ~1] in case of [ 0 Cars in our measurement Range ~ Max Number of Cars reported by Sensor Fusion Data].


* State Decision
  - Check the possible Lane Changes from Current Lane,
  - Check the Adjacent Lanes having Lowest cost,
  - Check the Safety Condition of adjacent Lane Changes.
  - Check for any Vehicle has got too close to us in front

* Output Command 
  - Control 'ref_vel' as per the speed of Vehicle in the front,
  - Command 'lane' variable for respective State Decision,
  - Wait for Lane Change.
  - Wait for Stabilization Phase after Lane Change.

### Trajectory Generation
Based on the desired intended behavior, the trajectory planning component will determine which trajectory is best for executing this behavior.
Trajectory generation code is inspired from project code walk through in course. It starts with finding any previous points and adding onto it based on following variables:

* **lane** Variable
* **ref_vel** Variable

we keep the refence x,y and yaw points (ref_x, ref_y and ref_yaw ).
Checking any previous points left and it is almost empty, then we use current car’s point to find the previous point and add them to the list.
If there is already previous points, then we just add previous two points. Also we are defining last known previous point to reference x and y.

Now we need to add 3 future points to ptsx, psy vecotrs. as car_s is frenet and we need to conver to the global x,y coordinates using getXY function. In total ptsx, ptsy has got 5 points in total each.

For trajectory generation, we are using spline instead of polynomial trajectory generation. We intialise the spline with 'ptsx' and 'ptsy'.
Then we add all previous points to 'next_x_vals' and 'next_y_vals' as it going to be the final control values pass it to the simulator and it will helps to get a smooth transition to the new points that we calculate later.

Now we need to find the all spline points till the horizon(say 30m) value so that spacing the way that ego car can travel at desired speed. Remeber the speed of the car is depend on the spacing between the points. If we know x point(ie 30m in this case), spline will be able to get us correponding spline y points.
![alt text][image4] 

We can calculate the spline points from start to horizon y points by using the formula mentioned in the picture. Our number of points has to be calculated is 50. We already added remaining previous points to the vectors next_x_vals and next_y_vals. We need to get the spline points of 50-previous_points.

next_x_vals and next_y_vals now have all 50 points which consist of 3 future points and 47 previous points.

## Rubric Points
### Compilation :  
    Code compiles with the cmake && make command. Only One new file Spline.h was added additionally for accessing Interpolation Library.
### Valid Trajectories :
    The Car is able to drive in the simulator satisfying following points:
    1. Incident free 4.32 miles Drive,
    2. Car always keeps Speed Limit in Check,
    3. Car always keeps Max Acceleration and Jerk in Limits.
    4. Car stays in Lane, except for the time changing the lane
    5. Car is able to change lane smoothly, efficiently and safely.
 ![alt text][image3] 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program
## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```