#ifndef PLANNER_H
#define PLANNER_H

#include <cassert>
#include <cmath>
#include <math.h>
#include <iostream>
#include <tuple>
#include <vector>
#include "spline.h"

using namespace std;

#define MILES_PER_HOUR_2_METERS_PER_SECOND 0.44704
#define MAX_ACCELERATION 10.

class Planner
{
  public:
    Planner() {}

    virtual ~Planner() {}

    /**
     * New path data
     */
    vector<double> next_path_x;
    vector<double> next_path_y;

    /**
     * Runs the planner to generate a new set of points for the car to follow.
     */
    void ExecutePlanner();

    /**
     * Initializes configuration data
     */
    void InitConfig(int nextPathSize, int maxLastPathReuseSize, double dt)
    {
        nextPathSize_ = nextPathSize;
        maxLastPathReuseSize_ = maxLastPathReuseSize;
        dt_ = dt;
    }

    /*
     * Initializes map data 
     */
    void InitMap(vector<double> &maps_x, vector<double> &maps_y,
                 vector<double> &maps_s, vector<double> &maps_dx, vector<double> &maps_dy);

    /**
     * Initializes road data
     */
    void InitRoad(int numberOfLanes, double laneWidth, double speedLimit, double roadVisibility);

    /**
     * Update cars data
     */
    void UpdateCarState(double car_x, double car_y, double car_s, double car_d,
                        double car_yaw, double car_speed);

    /**
     * Updates previous path data
     */
    void UpdatePreviousPathState(vector<double> &previous_path_x, vector<double> &previous_path_y,
                                 double previous_path_end_s, double previous_path_end_d);

    /**
     * Updates sensor fusion data
     */
    void UpdateSensorFusionState(vector<vector<double>> sensor_fusion);

  private:
    /**
     * Config data
     */
    // maximun number of points from previous path we will reuse
    // smaller number - more responsive to current road condition
    // bigger number - more smooth and less calculations
    int maxLastPathReuseSize_ = 10;

    // Number of points to return when we generate a path
    int nextPathSize_ = 50;

    // time difference between each point
    double dt_ = 0.02;

    // we multiple this to the maximum allowed acceleration
    // to control the rate of acceleration.
    double accelerationFactor_ = 0.5;

    int totalPointsTraveled = 0;

    /**
     * Map data
     */
    vector<double> map_x_;
    vector<double> map_y_;
    vector<double> map_s_;
    vector<double> map_dx_;
    vector<double> map_dy_;

    /**
     * Road data
     */
    int roadCount_;
    double roadWidth_;
    double roadSpeedLimit_;
    double roadVisibility_;

    /**
     * Road prediction data
     */
    // gap of the car in front in each lane
    vector<double> predictedRoadAheadGap_;
    // gap of the car behind in each lane
    vector<double> predictedRoadBehindGap_;
    // speed of the car in front in each lane
    vector<double> predictedRoadAheadSpeed_;
    // speed of the car behind in each lane
    vector<double> predictedRoadBehindSpeed_;

    // how fast we allow the car to go
    double max_v_;

    /**
     * Car current state data
     */
    double car_x_;
    double car_y_;
    double car_s_;
    double car_d_;
    double car_yaw_;
    double car_speed_; // car speed in miles per hour
    double car_v_;     // car velocity in meters per second

    // The car is currently in this lane.
    // 0 is the left most lane. roadCount_ - 1 is the right most lane.
    int car_lane_;

    /**
     * Car previous path data
     */
    vector<double> last_path_x_;
    vector<double> last_path_y_;
    double last_path_end_s_;
    double last_path_end_d_;
    int last_path_reuse_size_;

    /**
     * Sensor fusion data
     */
    vector<vector<double>> sensor_fusion_;

    /**
     * Reference points
     */
    double ref_x_;
    double ref_y_;
    double ref_yaw_;
    double ref_v_;
    double ref_d_;
    double ref_s_;
    int ref_lane_;

    int target_lane_;
    double target_v_;

    /**cd 
     * Adjusts the speed to target speed.
     */
    double AdjustSpeed(double current_v, double target_v);

    /**
     * Compute target speed for each lane
     */
    vector<double> ComputeLaneTargetSpeed();

    /**
     * Computes the reference state, this is the state the car is in at the first generated new point. 
     */
    void ComputeReferenceState();

    /**
     * Creates a spline from the reference point
     */
    tk::spline ComputeSpline();

    /**
     * Computes the lane given the frenet d value
     */
    int GetLane(double d);

    /**
     * Performs behavior planning.
     */
    void PerformBehaviorPlanning();

    /**
     * Performs prediction.
     */
    void PerformPrediction();

    /**
     * Performs trajectory generation.
     */
    void PerformTrajectoryGeneration();

    /**
     * Converts local coordinates back to global coordinates
     */
    static void ToGlobalCoordinates(
        vector<double> &x_points, vector<double> &y_points, double ref_x, double ref_y, double ref_yaw)
    {
        for (size_t i = 0; i < x_points.size(); i++)
        {
            double x = x_points[i];
            double y = y_points[i];

            x_points[i] = ref_x + (x * cos(ref_yaw) - y * sin(ref_yaw));
            y_points[i] = ref_y + (x * sin(ref_yaw) + y * cos(ref_yaw));
        }
    }

    /**
     * Converts global coordinates to local coordinates
     */
    static void ToLocalCoordinates(
        vector<double> &x_points, vector<double> &y_points, double ref_x, double ref_y, double ref_yaw)
    {
        // shift car reference angle to 0 degrees
        for (int i = 0; i < x_points.size(); i++)
        {
            // shift car reference angle to 0 degrees
            double shift_x = x_points[i] - ref_x;
            double shift_y = y_points[i] - ref_y;

            x_points[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            y_points[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
        }
    }
};

#endif