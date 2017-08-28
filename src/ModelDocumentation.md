# Project Path Planning
Path planning for high way driving

[Link to the video on youtube, car using the implemented path planner to drive around the track](https://youtu.be/zf6wu0zijbM)

## Project Structure
1. helper.h - The project supplied functions (e.g. getXY, getFrenet, etc) are refactored in this file
2. planner.h and planner.cpp - The path planner's implementation. This is the majority implementation of the project. More about it below.
3. main.cpp - This handles interaction with the simulator and planner. On startup, it will call the planner's InitMap, InitRoad and InitConfig functions to initialize the planner. On each cycle, it will call UpdateCarState, UpdatePreviousPathState, UpdateSensorFusionState functions to update the states. It will then call ExecutePlanner function go ask the planner to plan for next path. It will pass the next path to the simulator and the cycle repeats.


## Planner Architecture

### Initialization functions

#### InitMap - allows caller to initialize the map way point data

#### InitRoad - allows caller to initialize the road data
1. number of lanes on one side of the road
2. lane width 
3. speed limit of the road
4. visibility - how far our car should consider other vehicles on the road. Cars outside of this distance will not be considered when we plan our path.

#### InitConfig - allows caller to configure the planner
1. number of points to generate for next path; it's set to 50 in this project
2. number of points from last path to reuse; it's set to 50 in this project, meaning we are reusing all previous points
3. time elapses between frame; it's 0.02s for this project

### Update functions
#### UpdateCarState - update our car's state (e.g. position, speed)
#### UpdatePreviousPathState - update the previous path points not yet realized by the simulator
#### UpdateSensorFusionState - update the sensor fusion data

### ExecutePlanner - perform prediction, behavior planing and trajectory generation

#### Prediction -  PerformPrediction Function
This function performs prediction. Since this is highway driving prediction, the prediction is relatively simple. Here the only thing we are predicting are the locations of the other cars on the road at some time t later. 

We then combines this data to predict the closest car in front and behind in each lane in order to plan our furture path.

#### Behavior Planning - PerformBehaviorPlanning function
This function performs behavior planning. Using the predicted road information, this will determine whether the car should stay in its current length, or move into an adjacent lane. It will also determine the desired speed. The output of this is an actionable target lane and speed that the trajectory generation function can use.

#### Trajectory Generation - PerformTrajectoryGeneration
This function perform trajectory generation to achieve the target speed and lane. The trajectory generation uses the same method described in project walkthrough. It generates a spline and "pick" points from the spline to achieve desired velocity and acceleration.