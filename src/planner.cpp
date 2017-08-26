#include "planner.h"
#include <iostream>
#include <math.h>
#include <tuple>
#include "spline.h"
#include "helper.h"

using namespace std;

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

    cout << "Car reference v " << ref_v_ << endl;
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

int Planner::GetLane(double d)
{
    return d / this->roadWidth_;
}

tuple<vector<double>, vector<double>> Planner::GenerateTrajectory()
{
    vector<double> result_path_x;
    vector<double> result_path_y;

    // add points from previous path to reuse them
    // we are only going to reuse up to [last_path_reuse_size_] number of points
    result_path_x.insert(result_path_x.end(), last_path_x_.begin(), last_path_x_.begin() + last_path_reuse_size_);
    result_path_y.insert(result_path_y.end(), last_path_y_.begin(), last_path_y_.begin() + last_path_reuse_size_);

    // create a spline
    // The spline will help to minimize the jerk
    // By controlling the distance between the points we pick, we can control
    // the velocity
    tk::spline s = ComputeSpline();

    double target_x = 30.0;
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
    result_path_x.insert(result_path_x.end(), new_path_x.begin(), new_path_x.end());
    result_path_y.insert(result_path_y.end(), new_path_y.begin(), new_path_y.end());

    return make_tuple(result_path_x, result_path_y);
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
    roadCount_ = numberOfLanes;
    roadWidth_ = laneWidth;
    roadSpeedLimit_ = speedLimit;

    max_v_ = (roadSpeedLimit_ - 1.0) * MILES_PER_HOUR_2_METERS_PER_SECOND;
}

tuple<vector<double>, vector<double>> Planner::PlanPath()
{
    // calculate the reference state
    ComputeReferenceState();

    PlanTargetLaneAndSpeed();

    return GenerateTrajectory();
}

void Planner::PlanTargetLaneAndSpeed()
{
    target_lane_ = ref_lane_;
    target_v_ = ref_v_;
    target_v_ = max_v_;

    // stores speed and distance of the closest car in front in each lane
    vector<double> laneAheadGap;
    vector<double> laneBehindGap;
    vector<double> laneAheadSpeed;
    vector<double> laneBehindSpeed;
    for (size_t i = 0; i < roadCount_; i++)
    {
        laneAheadGap.push_back(999.0);
        laneBehindGap.push_back(999.0);
        laneAheadSpeed.push_back(max_v_);
        laneBehindSpeed.push_back(0.0);
    }

    // we are reusing some previous points, predict where this car will be
    // when our car finished executing the previous points
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

        double start_s = s + time_gap * v;

        double s_diff = fabs(start_s - ref_s_);

        // do not consider cars that are too far away
        if (s_diff > 200.0)
        {
            continue;
        }

        if (start_s > ref_s_)
        {
            // this car is in front
            if (s_diff < laneAheadGap[lane])
            {
                laneAheadGap[lane] = s_diff;
                laneAheadSpeed[lane] = v;
            }
        }
        else
        {
            // this car is behind
            if (s_diff < laneBehindGap[lane])
            {
                laneBehindGap[lane] = s_diff;
                laneBehindSpeed[lane] = v;
            }
        }
    }

    double safeDistance = 30.0;
    double stayInLaneIncentive = 4.0;

    // set best lane to current lane
    int bestLane = ref_lane_;
    double bestLaneSpeed = laneAheadSpeed[ref_lane_];
    double bestLaneGap = laneAheadGap[ref_lane_];
    if (laneAheadGap[ref_lane_] > safeDistance)
    {
        bestLaneSpeed = max_v_;
    }
    double bestLaneProjectedGap = laneAheadGap[ref_lane_] + stayInLaneIncentive + (bestLaneSpeed * dt_ * 50);

    // consider changing lane
    int leftLane = ref_lane_ - 1;
    int rightLane = ref_lane_ + 1;

    // if left lane is a valid lane
    if (leftLane >= 0)
    {
        // and it's safe to change
        // if (laneBehindGap[leftLane] >= 8 ||
        //     (laneBehindSpeed[leftLane] < ref_v_ && laneBehindGap[leftLane]))
        if (laneBehindGap[leftLane] >= 6 &&
            laneAheadGap[leftLane] >= 15)
        {
            double leftLaneSpeed = laneAheadSpeed[leftLane];
            double leftLaneGap = laneAheadGap[leftLane];
            if (leftLaneGap > safeDistance)
            {
                leftLaneSpeed = max_v_;
            }

            double leftLaneProjectedGap = leftLaneGap + leftLaneSpeed * dt_ * 50;
            if (leftLaneProjectedGap > bestLaneProjectedGap)
            {
                bestLane = leftLane;
                bestLaneSpeed = leftLaneSpeed;
                bestLaneProjectedGap = leftLaneProjectedGap;
            }
        }
    }
    // if right lane is a valid lane
    if (rightLane <= roadCount_ - 1)
    {
        // and it's safe to change lane
        // if (laneBehindGap[rightLane] >= 8 ||
        //     (laneBehindGap[rightLane] >= 4 && laneBehindSpeed[rightLane] < ref_v_))
        if (laneBehindGap[rightLane] >= 6 &&
            laneAheadGap[rightLane] >= 15)
        {
            double rightLaneSpeed = laneAheadSpeed[rightLane];
            double rightLaneGap = laneAheadGap[rightLane];
            if (rightLaneGap > safeDistance)
            {
                rightLaneSpeed = max_v_;
            }

            double rightLaneProjectedGap = rightLaneGap + rightLaneSpeed * dt_ * 50;
            if (rightLaneProjectedGap > bestLaneProjectedGap)
            {
                bestLane = rightLane;
                bestLaneSpeed = rightLaneSpeed;
                bestLaneProjectedGap = rightLaneProjectedGap;
            }
        }
    }

    target_lane_ = bestLane;
    target_v_ = bestLaneSpeed;

    cout << "target lane " << bestLane << " speed " << bestLaneSpeed << endl;
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

    cout << "Car traveling " << car_speed << " mi/hour toward " << car_yaw << " deg in lane " << car_lane_
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

    // determine the number of points to reuse from previous path
    last_path_reuse_size_ = min((int)last_path_x_.size(), maxLastPathReuseSize_);

    cout << "Last path size " << previous_path_x.size() << " reusing " << last_path_reuse_size_ << endl;
}

void Planner::UpdateSensorFusionState(vector<vector<double>> sensor_fusion)
{
    sensor_fusion_ = sensor_fusion;
}