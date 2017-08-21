#include "planner.h"
#include <iostream>
#include <math.h>
#include <tuple>
#include "spline.h"
#include "helper.h"

using namespace std;

double ref_vel = 0;

void Planner::ComputeReferencePoint()
{
    int prev_size = last_path_x_.size();

    if (prev_size >= 2)
    {
        ref_x_ = last_path_x_[prev_size - 1];
        ref_y_ = last_path_y_[prev_size - 1];

        // calculate yaw from previous points
        double prev_x = last_path_x_[prev_size - 2];
        double prev_y = last_path_y_[prev_size - 2];
        ref_yaw_ = atan2(ref_y_ - prev_y, ref_x_ - prev_x);
    }
    else
    {
        ref_x_ = car_x_;
        ref_y_ = car_y_;

        // calculate yaw by converting given car yaw to rad
        ref_yaw_ = deg2rad(car_yaw_);
    }
}

tk::spline Planner::ComputeSpline()
{
    vector<double> ptsx;
    vector<double> ptsy;

    // find a point before the reference point and add to spline
    int numberOfPreviousPoints = last_path_x_.size();
    double first_x;
    double first_y;
    if (numberOfPreviousPoints >= 2)
    {
        first_x = last_path_x_[numberOfPreviousPoints - 2];
        first_y = last_path_y_[numberOfPreviousPoints - 2];
    }
    else
    {
        first_x = car_x_ - cos(car_yaw_);
        first_y = car_y_ - sin(car_yaw_);
    }
    ptsx.push_back(first_x);
    ptsy.push_back(first_y);

    // add the reference point to the spline
    ptsx.push_back(ref_x_);
    ptsy.push_back(ref_y_);

    // create three points 30 meters apart and add to the spline
    vector<double> next_wp0 = getXY(car_s_ + 30, (2 + 4 * car_lane_), map_s_, map_x_, map_y_);
    vector<double> next_wp1 = getXY(car_s_ + 60, (2 + 4 * car_lane_), map_s_, map_x_, map_y_);
    vector<double> next_wp2 = getXY(car_s_ + 90, (2 + 4 * car_lane_), map_s_, map_x_, map_y_);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    ToLocalCoordinates(ptsx, ptsy, ref_x_, ref_y_, ref_yaw_);

    cout << "spline "
         << "[" << first_x << "," << first_y << "] "
         << "[" << ref_x_ << "," << ref_y_ << "] "
         << endl;

    tk::spline s;
    s.set_points(ptsx, ptsy);

    return s;
}

int Planner::GetLane(double d)
{
    return d / this->laneWidth_;
}

void Planner::InitMap(vector<double> &maps_x, vector<double> &maps_y,
                      vector<double> &maps_s, vector<double> &maps_dx, vector<double> &maps_dy)
{
    map_x_ = maps_x;
    map_y_ = maps_y;
    map_s_ = maps_s;
    map_dx_ = maps_dx;
    map_dy_ = maps_dy;
}

void Planner::InitRoad(int numberOfLanes, double laneWidth, double speedLimit)
{
    numberOfLanes_ = numberOfLanes;
    laneWidth_ = laneWidth;
    speedLimit_ = speedLimit;
}

bool Planner::IsInSameLane(int other_car_d)
{
    // The other car's width
    // Obviously not all car has the same width, but sensor fusion data does not contain car width,
    // for the purpose of the whether a car is in our lane or "partially" in our lane, we will assume
    // the other car's width
    const double car_width = 1.5;

    double left_edge_d = car_lane_ * laneWidth_ - car_width;
    double right_edge_d = car_lane_ * laneWidth_ + laneWidth_ + car_width;

    if (other_car_d >= left_edge_d && other_car_d <= right_edge_d)
    {
        return true;
    }
    return false;
}

tuple<vector<double>, vector<double>> Planner::PlanPath()
{

    // calculate the reference points
    this->ComputeReferencePoint();

    int prev_size = last_path_x_.size();
    //double ref_vel = 48.8;

    if (prev_size > 0)
    {
        car_s_ = last_path_end_s_;
    }
    bool too_close = false;

    // find ref_v to use
    for (int i = 0; i < sensor_fusion_.size(); i++)
    {
        // car is in my lane
        float d = sensor_fusion_[i][6];
        if (IsInSameLane(d))
        {

            double vx = sensor_fusion_[i][3];
            double vy = sensor_fusion_[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion_[i][5];

            check_car_s += ((double)prev_size * .02 * check_speed); //if using previus points can project s value out
            // check s value greater than our car and s gap
            if ((check_car_s > car_s_) && ((check_car_s - car_s_) < 30))
            {
                //ref_vel = 29.5; // mph
                cout << "car " << i << "is too close in same lane d " << d << endl;

                too_close = true;
            }
        }
    }

    if (too_close)
    {
        ref_vel -= 0.224;
    }
    else if (ref_vel < 49.5)
    {
        ref_vel += 0.224;
    }

    tk::spline s = ComputeSpline();

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double x_add_on = 0;

    double N = (target_dist / (.02 * ref_vel / 2.24));

    vector<double> new_x_vals;
    vector<double> new_y_vals;

    // fill up the rest of path planner
    for (int i = 0; i < 50 - prev_size; i++)
    {
        // if (too_close)
        // {
        //     ref_vel -= 0.224;
        // }
        // else if (ref_vel < 49.5)
        // {
        //     ref_vel += 0.224;
        // }
        // if (ref_vel < 0) {
        //     ref_vel = 0;
        // }

        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        new_x_vals.push_back(x_point);
        new_y_vals.push_back(y_point);
    }

    // convert new points back to global coordinates
    ToGlobalCoordinates(new_x_vals, new_y_vals, this->ref_x_, this->ref_y_, this->ref_yaw_);

    // combine and return resulting new path points
    vector<double> new_path_x;
    vector<double> new_path_y;

    // add previous points
    new_path_x.insert(new_path_x.end(), last_path_x_.begin(), last_path_x_.end());
    new_path_y.insert(new_path_y.end(), last_path_y_.begin(), last_path_y_.end());

    // add new points
    new_path_x.insert(new_path_x.end(), new_x_vals.begin(), new_x_vals.end());
    new_path_y.insert(new_path_y.end(), new_y_vals.begin(), new_y_vals.end());

    return make_tuple(new_path_x, new_path_y);
}

void Planner::UpdateCarState(double car_x, double car_y, double car_s, double car_d,
                             double car_yaw, double car_speed)
{
    car_x_ = car_x;
    car_y_ = car_y;
    car_s_ = car_s;
    car_d_ = car_d;
    car_yaw_ = car_yaw;
    car_speed_ = car_speed;

    // determine the lane the car is in
    car_lane_ = GetLane(car_d);

    cout << "Car traveling " << car_speed << " mi/hour toward " << car_yaw << " deg on lane " << car_lane_
         << " xy [" << car_x << "," << car_y << "] "
         << " ds [" << car_d << "," << car_s << "] "
         << endl;
}

void Planner::UpdatePreviousPathState(vector<double> &previous_path_x, vector<double> &previous_path_y,
                                      double previous_path_end_s, double previous_path_end_d)
{
    last_path_x_ = previous_path_x;
    last_path_y_ = previous_path_y;
    last_path_end_s_ = previous_path_end_s;
    last_path_end_d_ = previous_path_end_d;
}

void Planner::UpdateSensorFusionState(vector<vector<double>> sensor_fusion)
{
    sensor_fusion_ = sensor_fusion;
}