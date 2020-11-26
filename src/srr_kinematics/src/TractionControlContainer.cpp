#include "srr_kinematics/TractionControlContainer.hpp"

#include <cmath>
#include <algorithm>

bool SRR::TractionControlContainer::calculate_srr_linear_velocity(tf::Vector3& lin_vel)
{
    double x_dot = 0.0;

    if (curr_move_direction != MoveDirectionEnum::STATIONARY)
    {
        switch (curr_move_type)
        {
            case MoveTypeEnum::STRAIGHT:
            {
                x_dot = static_cast<int>(curr_move_direction)
                         * wheel_radius
                         * max_wheel_rate;
                break;
            }
            case MoveTypeEnum::TURN:
            {
                double Xfo  = longitudinal_distance_origin_front;
                double Xro  = longitudinal_distance_origin_rear;
                double dYof = curr_lateral_distance_origin_front + curr_dist_between_ICC_origin;
                double dYor = curr_lateral_distance_origin_rear  + curr_dist_between_ICC_origin;
                double turn_radius_front = std::sqrt((Xfo * Xfo) + (dYof * dYof));
                double turn_radius_rear  = std::sqrt((Xro * Xro) + (dYor * dYor));
                x_dot = static_cast<int>(curr_move_direction)
                         * wheel_radius
                         * max_wheel_rate
                         * (curr_dist_between_ICC_origin / std::max(turn_radius_front, turn_radius_rear));
                break;
            }
            default:
                return false;
        }
    }

    lin_vel.setValue(x_dot, 0, 0);
    return true;
}

SRR::TractionControlContainer::TractionControlContainer(
    double _wheel_radius,
    double _max_wheel_rate,
    double _longitudinal_distance_origin_front,
    double _longitudinal_distance_origin_rear
) :
    wheel_radius(_wheel_radius),
    max_wheel_rate(_max_wheel_rate),
    longitudinal_distance_origin_front(_longitudinal_distance_origin_front),
    longitudinal_distance_origin_rear(_longitudinal_distance_origin_rear)
{
}

bool SRR::TractionControlContainer::publish_wheel_rates()
{
    tf::Vector3 vehicle_linear_velocity;
    return calculate_srr_linear_velocity(vehicle_linear_velocity);
}
