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
                BogiePivotEnum bpYof;
                BogiePivotEnum bpYor;
                if (curr_move_type == MoveTypeEnum::TURN_LEFT)
                {
                    bpYof = BogiePivotEnum::RIGHT_NEAR;
                    bpYor = BogiePivotEnum::RIGHT_FAR;
                }
                else
                {
                    bpYof = BogiePivotEnum::LEFT_NEAR;
                    bpYor = BogiePivotEnum::LEFT_FAR;
                }
                double Xfo  = curr_longitudinal_distance_origin_axle;
                double Xro  = -curr_longitudinal_distance_origin_axle;
                double dYof = lateral_vector_origin_bogie_pivot.at(bpYof).y() + curr_dist_between_ICC_origin;
                double dYor = lateral_vector_origin_bogie_pivot.at(bpYor).y() + curr_dist_between_ICC_origin;
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

bool SRR::TractionControlContainer::estimate_contact_angles(tf::Vector3 lin_vel, BogieAbstractMap<double>& cont_ang)
{
    // WARNING: Assumes no rotation from vehicle origin to home position bogie joint, ensure that's correct
    for (int bp = static_cast<int>(BogiePivotEnum::LEFT_NEAR); static_cast<BogiePivotEnum>(bp) <= BogiePivotEnum::RIGHT_FAR; bp++)
    {
        BogiePivotEnum bpe = static_cast<BogiePivotEnum>(bp);
        tf::Vector3 ang_vel_bogie_frame_wrt_body(
            curr_vehicle_angular_velocity.x(),
            curr_vehicle_angular_velocity.y() + curr_bogie_angular_velocity.at(bpe),
            curr_vehicle_angular_velocity.y()
        );
        double cbp = curr_bogie_positions[bpe];
        std::cout << (bp - static_cast<int>(BogiePivotEnum::LEFT_NEAR)) << ":" << cbp << ", ";
        tf::Vector3 trans_bogie_to_wheel_frame_wrt_body(
            (bogie_leg_length * std::cos(cbp)) - (wheel_connection_link_length * std::sin(cbp)),
            0,
            (bogie_leg_length * std::sin(cbp)) + (wheel_connection_link_length * std::cos(cbp))
        );
        tf::Vector3 lin_vel_wheel_wrt_body = lin_vel + curr_vehicle_angular_velocity.cross(lateral_vector_origin_bogie_pivot.at(bpe))
                                                     + ang_vel_bogie_frame_wrt_body.cross(trans_bogie_to_wheel_frame_wrt_body);
        cont_ang[bpe] = -std::atan(lin_vel_wheel_wrt_body.z() / lin_vel_wheel_wrt_body.x());
    }
    return true;
}

SRR::TractionControlContainer::TractionControlContainer(
    double _wheel_radius,
    double _max_wheel_rate,
    double _bogie_leg_length,
    double _wheel_connection_link_length,
    std::string _bogie_pivot_joint_name_left_near,
    std::string _bogie_pivot_joint_name_right_near,
    std::string _bogie_pivot_joint_name_left_far,
    std::string _bogie_pivot_joint_name_right_far,
    double _lateral_distance_origin_bogie_pivot_left_near,
    double _lateral_distance_origin_bogie_pivot_right_near,
    double _lateral_distance_origin_bogie_pivot_left_far,
    double _lateral_distance_origin_bogie_pivot_right_far
) :
    wheel_radius(_wheel_radius),
    max_wheel_rate(_max_wheel_rate),
    bogie_leg_length(_bogie_leg_length),
    wheel_connection_link_length(_wheel_connection_link_length),
    bogie_pivot_joint_names({
        {_bogie_pivot_joint_name_left_near,  LEFT_NEAR},
        {_bogie_pivot_joint_name_right_near, RIGHT_NEAR},
        {_bogie_pivot_joint_name_left_far,   LEFT_FAR},
        {_bogie_pivot_joint_name_right_far,  RIGHT_FAR}
    }),
    lateral_vector_origin_bogie_pivot({
        {LEFT_NEAR,  tf::Vector3(0, _lateral_distance_origin_bogie_pivot_left_near,  0)},
        {RIGHT_NEAR, tf::Vector3(0, _lateral_distance_origin_bogie_pivot_right_near, 0)},
        {LEFT_FAR,   tf::Vector3(0, _lateral_distance_origin_bogie_pivot_left_far,   0)},
        {RIGHT_FAR,  tf::Vector3(0, _lateral_distance_origin_bogie_pivot_right_far,  0)}
    })
{
    curr_dist_between_ICC_origin = 0.5;
}

void SRR::TractionControlContainer::handle_joint_state_callback(const sensor_msgs::JointState& msg)
{
    for (std::size_t i = 0; i < msg.name.size(); i++)
    {
        const std::string name = msg.name[i];
        if (bogie_pivot_joint_names.find(name) != bogie_pivot_joint_names.end())
        {
            BogiePivotEnum bogie_pivot               = bogie_pivot_joint_names.at(name);
            curr_bogie_positions[bogie_pivot]        = msg.position[i];
            curr_bogie_angular_velocity[bogie_pivot] = msg.velocity[i];
        }
    }

    double bpln = curr_bogie_positions[BogiePivotEnum::LEFT_NEAR];
    curr_longitudinal_distance_origin_axle = (bogie_leg_length * std::cos(bpln)) - (wheel_connection_link_length * std::sin(bpln));
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
        std::cout << "Wz:" << msg.angular_velocity.z;
        //curr_move_type = (msg.angular_velocity.z > 0) ? MoveTypeEnum::TURN_LEFT : MoveTypeEnum::TURN_RIGHT;
        curr_move_type = MoveTypeEnum::STRAIGHT;
    }
    else
    {
        std::cout << "Wz:0";
        curr_move_type = MoveTypeEnum::STRAIGHT;
    }

    if (std::abs(msg.linear_acceleration.x) > LIN_ACC_EPS)
    {
        std::cout << ", Ax:" << msg.linear_acceleration.x;
        curr_move_direction = (msg.linear_acceleration.x > 0) ? MoveDirectionEnum::FORWARD : MoveDirectionEnum::BACKWARD;
    }
    else
    {
        std::cout << ", Ax:0";
        curr_move_direction = MoveDirectionEnum::STATIONARY;
    }
}

bool SRR::TractionControlContainer::publish_wheel_rates(BogieAbstractMap<double>& contact_angles)
{
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
