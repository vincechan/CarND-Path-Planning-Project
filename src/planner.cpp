#include "planner.h"
#include <iostream>
#include <math.h>
#include <tuple>
#include "spline.h"
#include "helper.h"

using namespace std;

double Planner::AdjustSpeed(double current_v, double target_v)
{
    //cout << " adjusting speed target " << target_v << " from " << current_v;

    double step_v = MAX_ACCELERATION * accelerationFactor_ * dt_;

    if (current_v < target_v)
    {
        // // adjust the acceleration when we are close to the limit
        // if (current_v + 0.2 >= max_v_) {
        //     step_v = step_v / 5;
        // }

        current_v += step_v;
        current_v = min(target_v, current_v);
    }
    else if (current_v > target_v)
    {
        current_v -= step_v;
        current_v = max(target_v, current_v);
    }

    //cout << " to " << current_v << endl;
    return current_v;
}

vector<double> Planner::ComputeLaneTargetSpeed()
{
}

void Planner::ComputeReferenceState()
{
    if (last_path_reuse_size_ >= 2)
    {
        ref_x_ = last_path_x_[last_path_reuse_size_ - 1];
        ref_y_ = last_path_y_[last_path_reuse_size_ - 1];

        // calculate yaw from previous points
        double prev_x = last_path_x_[last_path_reuse_size_ - 2];
        double prev_y = last_path_y_[last_path_reuse_size_ - 2];
        ref_yaw_ = atan2(ref_y_ - prev_y, ref_x_ - prev_x);

        // calculate velocity from previous points
        ref_v_ = distance(ref_x_, ref_y_, prev_x, prev_y) / 0.02;

        vector<double> sd = getFrenet(ref_x_, ref_y_, ref_yaw_, map_x_, map_y_);
        ref_s_ = sd[0];
        ref_d_ = sd[1];
    }
    else
    {
        ref_x_ = car_x_;
        ref_y_ = car_y_;
        ref_yaw_ = deg2rad(car_yaw_);
        ref_v_ = car_speed_ * MILES_PER_HOUR_2_METERS_PER_SECOND;

        ref_d_ = car_d_;
        ref_s_ = car_s_;
    }

    // determine the lane the car is in
    ref_lane_ = GetLane(ref_d_);
}

tk::spline Planner::ComputeSpline()
{
    vector<double> ptsx;
    vector<double> ptsy;

    // find a point before the reference point and add to spline
    double prev_x;
    double prev_y;
    if (last_path_reuse_size_ >= 2)
    {
        // take a point from last path
        prev_x = last_path_x_[last_path_reuse_size_ - 2];
        prev_y = last_path_y_[last_path_reuse_size_ - 2];
    }
    else
    {
        // compute it from car state
        prev_x = car_x_ - cos(car_yaw_);
        prev_y = car_y_ - sin(car_yaw_);
    }
    ptsx.push_back(prev_x);
    ptsy.push_back(prev_y);

    // add the reference point to the spline
    ptsx.push_back(ref_x_);
    ptsy.push_back(ref_y_);

    // TODO: use ref lane
    // create three points 30 meters apart and add to the spline
    vector<double> next_wp0 = getXY(ref_s_ + 30, (2 + 4 * target_lane_), map_s_, map_x_, map_y_);
    vector<double> next_wp1 = getXY(ref_s_ + 60, (2 + 4 * target_lane_), map_s_, map_x_, map_y_);
    vector<double> next_wp2 = getXY(ref_s_ + 90, (2 + 4 * target_lane_), map_s_, map_x_, map_y_);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    ToLocalCoordinates(ptsx, ptsy, ref_x_, ref_y_, ref_yaw_);

    tk::spline s;
    s.set_points(ptsx, ptsy);

    return s;
}

void Planner::ExecutePlanner()
{
    ComputeReferenceState();

    PerformPrediction();

    PerformBehaviorPlanning();

    PerformTrajectoryGeneration();
}

int Planner::GetLane(double d)
{
    return d / this->roadWidth_;
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

void Planner::InitRoad(int numberOfLanes, double laneWidth, double speedLimit, double visibility)
{
    roadCount_ = numberOfLanes;
    roadWidth_ = laneWidth;
    roadSpeedLimit_ = speedLimit;
    roadVisibility_ = visibility;

    max_v_ = (roadSpeedLimit_ - 0.15) * MILES_PER_HOUR_2_METERS_PER_SECOND;
}

void Planner::PerformBehaviorPlanning()
{
    target_lane_ = ref_lane_;
    target_v_ = max_v_;

    double safeChangeLaneDistance = 9.0;

    vector<double> laneScore;
    for (size_t i = 0; i < roadCount_; i++)
    {
        laneScore.push_back(0);
    }

    for (size_t i = 0; i < roadCount_; i++)
    {

        // check whether it's safe to be in this lane
        // TODO: use speed to determine whether the lane is safe
        if (i == ref_lane_)
        {
            // we are already in this lane
        }
        else if (i == ref_lane_ - 2)
        {
            // left 2 lane
            if (!(predictedRoadBehindGap_[ref_lane_ - 1] >= safeChangeLaneDistance &&
                  predictedRoadAheadGap_[ref_lane_ - 1] >= safeChangeLaneDistance &&
                  predictedRoadBehindGap_[ref_lane_ - 2] >= safeChangeLaneDistance * 2 &&
                  predictedRoadAheadGap_[ref_lane_ - 2] >= safeChangeLaneDistance * 2))
            {
                continue;
            }
        }
        else if (i == ref_lane_ - 1)
        {
            // left 1 lane
            if (!(predictedRoadBehindGap_[ref_lane_ - 1] >= safeChangeLaneDistance &&
                  predictedRoadAheadGap_[ref_lane_ - 1] >= safeChangeLaneDistance))
            {
                continue;
            }
        }
        else if (i == ref_lane_ + 1)
        {
            // right 1 lane
            if (!(predictedRoadBehindGap_[ref_lane_ + 1] >= safeChangeLaneDistance &&
                  predictedRoadAheadGap_[ref_lane_ + 1] >= safeChangeLaneDistance))
            {
                continue;
            }
        }
        else if (i == ref_lane_ + 2)
        {
            // right 2 lane
            if (!(predictedRoadBehindGap_[ref_lane_ + 1] >= safeChangeLaneDistance &&
                  predictedRoadAheadGap_[ref_lane_ + 1] >= safeChangeLaneDistance &&
                  predictedRoadBehindGap_[ref_lane_ + 2] >= safeChangeLaneDistance * 2 &&
                  predictedRoadAheadGap_[ref_lane_ + 2] >= safeChangeLaneDistance * 2))
            {
                continue;
            }
        }
        else
        {
            // do not consider more than 2 lanes away
            // note, not possible in this project as we only have 3 lanes on the road.
            continue;
        }

        double speed = predictedRoadAheadSpeed_[i];
        double gap = predictedRoadAheadGap_[i];
        if (gap <= 10)
        {
            // slow down to slower than the car in front
            speed = speed - 10;
        }
        else if (gap >= 30)
        {
            speed = max_v_;
        }
        else
        {
            speed = speed + 1; // close in the gap
        }

        int numberOfLaneChanges = ref_lane_ - i;
        if (numberOfLaneChanges < 0)
        {
            numberOfLaneChanges = numberOfLaneChanges * -1;
        }

        // calculate the lane score using a combination of available gap, change lane cost, and speed
        double gapScore = min(gap, 100.0) + gap / 100.0;
        double laneChangeScore = numberOfLaneChanges * 6 * -1;
        double speedScore = 0;
        if (gap < 40)
        {
            speedScore = (ref_v_ - speed) * 2;
        }
        laneScore[i] = gapScore + laneChangeScore + laneChangeScore;
    }

    int bestLane = ref_lane_;
    double bestlaneScore = laneScore[ref_lane_];
    for (size_t i = 0; i < roadCount_; i++)
    {
        if (laneScore[i] > bestlaneScore)
        {
            bestlaneScore = laneScore[i];
            bestLane = i;
        }
    }

    int laneChange = bestLane - ref_lane_;
    if (laneChange > 1 || laneChange < -1)
    {
        // we need to change more than 1 lane
        // the trajectory generation couldn't handle this without violating acceleration limit in the simulator
        // we will suggest changing one lane first
        target_lane_ = 1;
    }
    else
    {
        target_lane_ = bestLane;
    }

    if (predictedRoadAheadGap_[ref_lane_] <= 8)
    {
        // slow down to slower than the car in front
        target_v_ = predictedRoadAheadSpeed_[ref_lane_] - 5;
    }
    else if (predictedRoadAheadGap_[ref_lane_] >= 24)
    {
        target_v_ = max_v_;
    }
    else
    {
        target_v_ = predictedRoadAheadSpeed_[ref_lane_] + 1;
    }

    // if the traffic on the road is faster than speed limit, we will stick with speed limit
    if (target_v_ > max_v_)
    {
        target_v_ = max_v_;
    }

    // adjust how hard we accelerate depends on whether we are changing lane
    if (ref_lane_ == target_lane_)
    {
        accelerationFactor_ = 0.9;
    }
    else
    {
        accelerationFactor_ = 0.3;
    }

    // initial start up optimization
    // when the car first start, use close to max acceleration
    if (totalPointsTraveled <= 120)
    {
        target_lane_ = ref_lane_;
        accelerationFactor_ = 0.9;
    }

    // print out behavior information
    if (target_lane_ == ref_lane_)
    {
        cout << "KEEP LANE ";
    }
    else if (target_lane_ < ref_lane_)
    {
        cout << "CHANGE LEFT ";
    }
    else
    {
        cout << "CHANGE RIGHT ";
    }
    cout << target_lane_
         << " current s " << ref_v_
         << " target s " << target_v_
         << " points " << totalPointsTraveled
         << " last " << last_path_reuse_size_
         << endl;
}

void Planner::PerformPrediction()
{
    // Perform prediction of other cars on the road
    //
    // We are only predicting the closest car in front and behind in each lane,
    // not every cars visible
    //
    // We predict other cars at some time after we recevie the sensor fusion data.
    // Specifically, we are reusing points from previous path, our prediction starts
    // when the previous path points end.

    predictedRoadAheadGap_.clear();
    predictedRoadBehindGap_.clear();
    predictedRoadAheadSpeed_.clear();
    predictedRoadBehindSpeed_.clear();

    for (size_t i = 0; i < roadCount_; i++)
    {
        predictedRoadAheadGap_.push_back(999.0);
        predictedRoadBehindGap_.push_back(999.0);
        predictedRoadAheadSpeed_.push_back(max_v_);
        predictedRoadBehindSpeed_.push_back(0.0);
    }

    // compute prediction start time
    double time_gap = dt_ * last_path_reuse_size_;

    for (size_t i = 0; i < sensor_fusion_.size(); i++)
    {
        double vx = sensor_fusion_[i][3];
        double vy = sensor_fusion_[i][4];
        double s = sensor_fusion_[i][5];
        float d = sensor_fusion_[i][6];
        float lane = GetLane(d);

        // compute speed
        double v = sqrt(vx * vx + vy * vy);

        // compute prediction start s
        double start_s = s + time_gap * v;

        // compute the s difference between this car and our car
        double s_diff = fabs(start_s - ref_s_);

        // do not consider cars that are too far away
        if (s_diff > roadVisibility_)
        {
            continue;
        }

        // if this car is in front
        if (start_s > ref_s_)
        {
            // and this car is closer
            if (s_diff < predictedRoadAheadGap_[lane])
            {
                // update prediction
                predictedRoadAheadGap_[lane] = s_diff;
                predictedRoadAheadSpeed_[lane] = v;
            }
        }
        else
        {
            // this car is behind
            if (s_diff < predictedRoadBehindGap_[lane])
            {
                predictedRoadBehindGap_[lane] = s_diff;
                predictedRoadBehindSpeed_[lane] = v;
            }
        }
    }
}

void Planner::PerformTrajectoryGeneration()
{
    next_path_x.clear();
    next_path_y.clear();

    // add points from previous path to reuse them
    // we are only going to reuse up to [last_path_reuse_size_] number of points
    next_path_x.insert(next_path_x.end(), last_path_x_.begin(), last_path_x_.begin() + last_path_reuse_size_);
    next_path_y.insert(next_path_y.end(), last_path_y_.begin(), last_path_y_.begin() + last_path_reuse_size_);

    // create a spline
    // The spline will help to minimize the jerk
    // By controlling the distance between the points we pick, we can control
    // the velocity and acceleration
    tk::spline s = ComputeSpline();

    double target_x = 50.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double x_add_on = 0;
    double v = ref_v_;

    vector<double> new_path_x;
    vector<double> new_path_y;

    for (int i = 0; i < nextPathSize_ - last_path_reuse_size_; i++)
    {
        v = AdjustSpeed(v, target_v_);

        double N = (target_dist / (dt_ * v));
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        new_path_x.push_back(x_point);
        new_path_y.push_back(y_point);
    }
    // convert new points back to global coordinates
    ToGlobalCoordinates(new_path_x, new_path_y, this->ref_x_, this->ref_y_, this->ref_yaw_);

    // add the newly generated points to the resulting path
    next_path_x.insert(next_path_x.end(), new_path_x.begin(), new_path_x.end());
    next_path_y.insert(next_path_y.end(), new_path_y.begin(), new_path_y.end());
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

    car_v_ = car_speed_ * MILES_PER_HOUR_2_METERS_PER_SECOND;

    // determine the lane the car is in
    car_lane_ = GetLane(car_d);

    // cout << "Car traveling " << car_speed << " mi/hour toward " << car_yaw << " deg in lane " << car_lane_
    //      << " xy [" << car_x << "," << car_y << "] "
    //      << " ds [" << car_d << "," << car_s << "] "
    //      << endl;
}

void Planner::UpdatePreviousPathState(vector<double> &previous_path_x, vector<double> &previous_path_y,
                                      double previous_path_end_s, double previous_path_end_d)
{
    last_path_x_ = previous_path_x;
    last_path_y_ = previous_path_y;
    last_path_end_s_ = previous_path_end_s;
    last_path_end_d_ = previous_path_end_d;

    // determine the number of points to reuse from previous path
    last_path_reuse_size_ = min((int)last_path_x_.size(), maxLastPathReuseSize_);

    totalPointsTraveled += (nextPathSize_ - last_path_x_.size());
}

void Planner::UpdateSensorFusionState(vector<vector<double>> sensor_fusion)
{
    sensor_fusion_ = sensor_fusion;
}