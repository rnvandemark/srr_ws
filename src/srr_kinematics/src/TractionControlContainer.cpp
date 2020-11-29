#include "srr_kinematics/TractionControlContainer.hpp"

#include <cmath>
#include <algorithm>

bool SRR::TractionControlContainer::calculate_vehicle_linear_velocity(tf::Vector3& lin_vel)
{
    bool rv = false;
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
                rv = true;
                break;
            }
            case MoveTypeEnum::TURN_LEFT:
            case MoveTypeEnum::TURN_RIGHT:
            {
                LegPivotEnum lpYof;
                LegPivotEnum lpYor;
                if (curr_move_type == MoveTypeEnum::TURN_LEFT)
                {
                    lpYof = LegPivotEnum::RIGHT_NEAR;
                    lpYor = LegPivotEnum::RIGHT_FAR;
                }
                else
                {
                    lpYof = LegPivotEnum::LEFT_NEAR;
                    lpYor = LegPivotEnum::LEFT_FAR;
                }
                double Xfo  = curr_longitudinal_distance_origin_axle;
                double Xro  = -curr_longitudinal_distance_origin_axle;
                double dYof = lateral_vector_origin_leg_pivot.at(lpYof).y() + curr_dist_between_ICC_origin;
                double dYor = lateral_vector_origin_leg_pivot.at(lpYor).y() + curr_dist_between_ICC_origin;
                double turn_radius_front = std::sqrt((Xfo * Xfo) + (dYof * dYof));
                double turn_radius_rear  = std::sqrt((Xro * Xro) + (dYor * dYor));
                x_dot = static_cast<int>(curr_move_direction)
                         * wheel_radius
                         * max_wheel_rate
                         * (curr_dist_between_ICC_origin / std::max(turn_radius_front, turn_radius_rear));
                rv = true;
                break;
            }
            default:
                break;
        }
    }

    lin_vel.setValue(x_dot, 0, 0);
    return rv;
}

bool SRR::TractionControlContainer::estimate_contact_angles(tf::Vector3 lin_vel, LegAbstractMap<double>& cont_ang)
{
    // WARNING: Assumes no rotation from vehicle origin to home position leg pivot joint, ensure that's correct
    int lp_max = static_cast<int>(LegPivotEnum::RIGHT_FAR);
    for (int lp = static_cast<int>(LegPivotEnum::LEFT_NEAR); lp <= lp_max; lp++)
    {
        LegPivotEnum lpe = static_cast<LegPivotEnum>(lp);
        tf::Vector3 ang_vel_leg_pivot_frame_wrt_body(
            curr_vehicle_angular_velocity.x(),
            curr_vehicle_angular_velocity.y() + curr_leg_pivot_angular_velocity.at(lpe),
            curr_vehicle_angular_velocity.y()
        );

        double alp = 0.0;
        for (double p : recent_leg_pivot_positions[lpe])
        {
            alp += p;
        }
        alp /= leg_pivot_positions_window_size;

        tf::Vector3 trans_leg_pivot_to_wheel_frame_wrt_body(
            (leg_length * std::cos(alp)) - (wheel_connection_link_length * std::sin(alp)),
            0,
            (leg_length * std::sin(alp)) + (wheel_connection_link_length * std::cos(alp))
        );
        tf::Vector3 lin_vel_wheel_wrt_body = lin_vel + curr_vehicle_angular_velocity.cross(lateral_vector_origin_leg_pivot.at(lpe))
                                                     + ang_vel_leg_pivot_frame_wrt_body.cross(trans_leg_pivot_to_wheel_frame_wrt_body);
        cont_ang[lpe] = -std::atan(lin_vel_wheel_wrt_body.z() / lin_vel_wheel_wrt_body.x());
    }
    return true;
}

SRR::TractionControlContainer::TractionControlContainer(
    double _wheel_radius,
    double _max_wheel_rate,
    double _leg_length,
    double _wheel_connection_link_length,
    std::string _leg_pivot_joint_name_left_near,
    std::string _leg_pivot_joint_name_right_near,
    std::string _leg_pivot_joint_name_left_far,
    std::string _leg_pivot_joint_name_right_far,
    double _lateral_distance_origin_leg_pivot_left_near,
    double _lateral_distance_origin_leg_pivot_right_near,
    double _lateral_distance_origin_leg_pivot_left_far,
    double _lateral_distance_origin_leg_pivot_right_far,
    int _leg_pivot_positions_window_size
) :
    wheel_radius(_wheel_radius),
    max_wheel_rate(_max_wheel_rate),
    leg_length(_leg_length),
    wheel_connection_link_length(_wheel_connection_link_length),
    leg_pivot_positions_window_size(_leg_pivot_positions_window_size),
    leg_pivot_joint_names({
        {_leg_pivot_joint_name_left_near,  LEFT_NEAR},
        {_leg_pivot_joint_name_right_near, RIGHT_NEAR},
        {_leg_pivot_joint_name_left_far,   LEFT_FAR},
        {_leg_pivot_joint_name_right_far,  RIGHT_FAR}
    }),
    lateral_vector_origin_leg_pivot({
        {LEFT_NEAR,  tf::Vector3(0, _lateral_distance_origin_leg_pivot_left_near,  0)},
        {RIGHT_NEAR, tf::Vector3(0, _lateral_distance_origin_leg_pivot_right_near, 0)},
        {LEFT_FAR,   tf::Vector3(0, _lateral_distance_origin_leg_pivot_left_far,   0)},
        {RIGHT_FAR,  tf::Vector3(0, _lateral_distance_origin_leg_pivot_right_far,  0)}
    })
{
    curr_dist_between_ICC_origin = 0.5;
}

void SRR::TractionControlContainer::handle_joint_state_callback(const sensor_msgs::JointState& msg)
{
    for (std::size_t i = 0; i < msg.name.size(); i++)
    {
        const std::string name = msg.name[i];
        if (leg_pivot_joint_names.find(name) != leg_pivot_joint_names.end())
        {
            LegPivotEnum lp = leg_pivot_joint_names.at(name);
            recent_leg_pivot_positions[lp].push_back(msg.position[i]);
            while (recent_leg_pivot_positions[lp].size() > leg_pivot_positions_window_size)
            {
                recent_leg_pivot_positions[lp].pop_front();
            }
            curr_leg_pivot_angular_velocity[lp] = msg.velocity[i];
        }
    }

    double lpln = recent_leg_pivot_positions[LegPivotEnum::LEFT_NEAR].front();
    curr_longitudinal_distance_origin_axle = (leg_length * std::cos(lpln)) - (wheel_connection_link_length * std::sin(lpln));
}

void SRR::TractionControlContainer::handle_imu_callback(const sensor_msgs::Imu& msg)
{
    curr_vehicle_angular_velocity.setValue(
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z
    );

    if (std::abs(msg.angular_velocity.z) > ANG_VEL_EPS)
    {
        curr_move_type = (msg.angular_velocity.z > 0) ? MoveTypeEnum::TURN_LEFT : MoveTypeEnum::TURN_RIGHT;
    }
    else
    {
        curr_move_type = MoveTypeEnum::STRAIGHT;
    }

    if (std::abs(msg.linear_acceleration.x) > LIN_ACC_EPS)
    {
        curr_move_direction = (msg.linear_acceleration.x > 0) ? MoveDirectionEnum::FORWARD : MoveDirectionEnum::BACKWARD;
    }
    else
    {
        curr_move_direction = MoveDirectionEnum::STATIONARY;
    }
}

bool SRR::TractionControlContainer::publish_wheel_rates(LegAbstractMap<double>& contact_angles)
{
    if (recent_leg_pivot_positions[LegPivotEnum::LEFT_NEAR].size() != leg_pivot_positions_window_size)
    {
        return false;
    }

    tf::Vector3 vehicle_linear_velocity;
    if (!calculate_vehicle_linear_velocity(vehicle_linear_velocity))
    {
        return false;
    }
    if (!estimate_contact_angles(vehicle_linear_velocity, contact_angles))
    {
        return false;
    }
    return true;
}
