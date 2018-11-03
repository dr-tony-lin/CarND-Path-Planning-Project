# Path Planning Project

## Content of the Submission and Usage

This submission includes the following c++ files:
* main.cpp: the main function that communicates with the simulator and drive the MPC process. It was modified from the original [CarND-Path-Planning-Propject](https://github.com/udacity/CarND-Path-Planning-Project)
* control/Navigator.[hpp, cpp: The class containing the path planning control
* control/Cost.[h, cpp]: Provide cost function implementation
* model/Vehicle.[hpp, cpp]: a vehicle model class that encapsulates vehicle states and vechicle related utility functions 
* model/Road.[hpp, cpp]: model road, and provide smooth mapping between map coordinate and Frenet coordinate
* utils/Config.[hpp, cpp]: the Config class for providing hyper parameters from config.json file
* utils/utils.[h, cpp]: utility functions

### Usage

The path planning executable can be launched with the following command:

    ./path_planning

The program will listen on port 4567 for an incoming simulator connection. Only one simulator should be connected at anytime, though the program does not prohibit it. To start a new simulator, terminate the existing one first, then start a new one.

To start the simulator:

#### On Windows

    term3_sim.exe

#### Other platforms:

    ./term3_sim

#### Build

For Windows, Bash on Ubuntu on Windows should be used. Both gcc and clang can be used to build the program.

To build the program, invoke the following commands on the bash terminal:
```
mkdir build
cd build
cmake ..
make
```

The program can be built to output more different level of information for disgnosis purposes by defining **VERBOSE_OUT**, **DEBUG_OUT**, **INFO_OUT**, and **DEBUG_OUT** macros.

#### Build API Documentation

The documentation for functions, classes and methods are included in the header files in Doxygen format. To generate Api documentation with the included doxygen.cfg:

1. Make sure doxygen is installed
2. cd to the root folder of this project
3. Issue the following command:

    doxygen doxygen.cfg

## The Implementation

### The source code structure

The src folder contains main.cpp, the following folders:

* control: contains the Navigator class, and the Cost function classes
* model: contains the Vehicle and Road models
* utils: contains utility classes and functions, the json and spline utilities classes

#### File names

In this project, I made a class to have its own .hpp, and .cpp files. More specifically, a class source file contains exactly one class, and has the same name as the class it contains.

#### The External folder

The external folder contains Eigen.

## Navigator class

This class provides the overall path planning control. It's process is as follows:

1. Before starting the navigation, initializeVehicle() should be called first to initialize the vehicle state.
2. During each control loop, invoke the update() method to update the trajectory returned from the simulator
3. Call the navigate() method to generate the next trajectory. THe Navigate method will perform the followings:
    * Complete the current state transition if the end of transition has been reach
    * Otherwise, get the states that can be transitted from the current state
        * Find the best state by evaluating the available states' cost using the CostEvaluator class
        * TRansit to the state
    * Compute the next trajectory

### The state
States and state transitions are defined in the State, and StateTransition classes respectively. THe following states are used:

* KL: keep lane
* LCL: change to left
* LCR: change to right
* COLLIIDED: the vehicle has involved in a collision, the program will exit at this point

Prepare lane change states are not used in this project as it is less important using a simulator.

### The Cost Function

The following cost functions (implemented as functor derived from Cost class) are used:

* SafetyCost: this functor penalize unsafe situation where there is not enough space to change lane
* SpeedCost: this functor penalize slow or fast approaching vehicle from behind
* OffTargetLaneCost: this functor penalize distance to the target lane
* ChangeLaneCost: this functor penalize lane change
* LaneTrafficCost: this cost functor peanlize lanes that has heavier traffic, especially vehicle ahead of the vehicle

These functors are encapsulated by CostEvaluator which is also itself a Cost functor. Also most of them use sigmoid as the underline cost function.

### Determing Maximum Acceleration

To minimize jerk, instead of controlling velocity directory, we compute the maximum permissible accelertion. The maximum acceleration is determined by using the following factors:

1. Vehicles in the collision course at the event horizon. I do not check every time step alone the horizon, but this is probably yield better result. If there is no vehicle, the max acceleration is assumed.
2. Otherwise, we compute the vehicle in front and at the back of the vehicle, and compute a suitable acceleration from their speed. It is possible that the vehicle behind might have a faster speed than the vehicle in front. This is left for the path planning to deal with (through the SafetyCost functor)

### Trajectory Generation

For smooth trajector generation, in stead of compute smooth curve using spline function for every trajectory, the following approach is used:

#### Spline Curves For The Road Geometry

1. The Road class will initialize two spline curves, one, SX, from the S coordinate to the X coordinate, and another, YS, from the S coordinate to the Y coordinate. Since the mapping from S to a map point is unique. This is also true for S to X, and S to Y. Having these two spline curved initialized when the Road class is initialized, we can obtain the X and Y coordinate of the waypoint given the S coordinate. 

2. We perform trajectory computation and collision detection in the Frenet coordinate. The obtain the waypoint from the resulting S coordinate. The vehicle's trajectory can then be obtained by adding the X, Y coordinate with the value of the D coordinate multiple by the normal of the road. e.g:

````
    [X,Y] = [SX(S), SY(s)] + D * N(s)

````


3. For smooth land change, a spline curve is used. It has the following control points:

````
std::vector<double> X = {0.0, 0.05, 0.5, 0.95, 1.0}
std::vector<double> Y = {0.0, 0.0,  0.5,  1.0, 1.0}
````
The spline is compute once when the navigation starts, it is then used for all subsequent lane change trajectory generation.
Given a lane change distance, DS, and lane travelling distance, DD, the D coordinate can be obtained from the S coordinate by scaing gthe curve by DS, DD.

### Normalizing S Coordinate

Since the road is circular, the S coordinate needs to be normalized to [0, length of road]. This normalization has also to be allpied in distance calculation.

Furthermore, for the spline curve, SX, and SY, we need to add the last waypoint to the begining of the curves' control points, and add the first point to the end of the curves' control points. However, to have second order continuity at the begining, two points from the one end are added to the other end.

## Results

The following video was taken before the submission to demostrate the vehicle navigation using the implemented path planner:

[Path Planning Demo] (self_driving_car_nanodegree_program 10_29_2018 8_27_42 PM.mp4)

It is to be noted that there are many factors that might affect the results. For the program to run reasonably, some reasonable hardware might be needed. Also on Windows 10, the simulator could run into issues. This happened once in my case that took me hours to figure that the simulator was the issue. I had to remove the simulator, and unzip it from the zip file again for it to work again.

## Discussion

I could use the same approach suggected in the project material, but it is interesting to see if spline work well as well. This tool me extra time to work on the project. But the result is very positive for me to conclude that it's feasibility.

###  The advantage of using spline curve for the road's waypoints

This approach has the following advantages over other methods:

1. There is no need for map to/from local coordinate (vehicle) transformations for computing the vechicle's trajectotry.
2. The approach is more efficient for mapping S back to XY coordinate. In my tests with 10 Million operations, the spline's approach took 593 millieseconds while the original approach included in main.cpp took 696 millieseconds.
3. For computing normal, the spline approach seems produce smoother result as it is second order continueous.
4. In computing the waypoints for asmooth trajectory, this approach also yield constant velocity comparing with the approach used in the Q&A which computes the waypoints in the vehicle's local coordinate system using spline. Even though its result will still be better than using the map's coordinate, but there will be still some level of distoration unless the spline is a straight line.

### The Challenge With the Spline Approach

The biggest challenge is the algorithm may be different from the algorithm the simulator is using. This results in bigger error terms that might sometime caused the simulator to treat the generated trajectory as having incidence. This took me extra time to tune the parameters for satisfactory results. The simple approach that I took to reduce incidences is to uae lower speed and acceleration limits.

### TO DO

Having implemented a very simple path planner my self, I ganed appreciation on how difficult and how much works it is to put an autonomous vehicle on the road. There are much more to learn, to experiment, and to do. Keep learning and have fun. 