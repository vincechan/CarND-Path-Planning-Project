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

#define MILES2METERS 1609.34
#define HOURS2SECONDS 1.0 / 3600
#define MAX_ACCELERATION 10.O

class Planner
{
  public:
    Planner() {}

    virtual ~Planner() {}

    tuple<vector<double>, vector<double>> PlanPath();

    /*
     * Initialize map data 
     */
    void InitMap(vector<double> &maps_x, vector<double> &maps_y,
                 vector<double> &maps_s, vector<double> &maps_dx, vector<double> &maps_dy);

    /**
     * Initialize road data
     */
    void InitRoad(int numberOfLanes, double laneWidth, double speedLimit);

    /**
     * Update car data
     */
    void UpdateCarState(double car_x, double car_y, double car_s, double car_d,
                        double car_yaw, double car_speed);

    /**
     * Update previous path data
     */
    void UpdatePreviousPathState(vector<double> &previous_path_x, vector<double> &previous_path_y,
                                 double previous_path_end_s, double previous_path_end_d);

    /**
     * Update sensor fusion data
     */
    void UpdateSensorFusionState(vector<vector<double>> sensor_fusion);

  private:
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
    int numberOfLanes_;
    double laneWidth_;
    double speedLimit_;

    /**
     * Car current state data
     */
    double car_x_;
    double car_y_;
    double car_s_;
    double car_d_;
    double car_yaw_;
    double car_speed_;
    // The car is currently in this lane.
    // 0 is the left most lane. numberOfLanes_ - 1 is the right most lane.
    int car_lane_;

    /**
     * Car previous path data
     */
    vector<double> last_path_x_;
    vector<double> last_path_y_;
    double last_path_end_s_;
    double last_path_end_d_;

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

    /**
     * Compute the reference point. This is the first point in our new path
     */
    void ComputeReferencePoint();

    /**
     * Create a spline from the reference point
     */
    tk::spline ComputeSpline();

    /**
     * Return the a car is in given the d value
     */
    int GetLane(double d);

    /**
     * Return whehter another car is in the same lane given it's d value.
     */
    bool IsInSameLane(int other_car_d);

    /**
     * convert local coordinates back to global coordinates
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
     * convert global coordinates to local coordinates
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