#include "srr_kinematics/TractionControlContainer.hpp"

#include "srr_utils/math_utils.hpp"

#include <cmath>
#include <algorithm>

bool SRR::TractionControlContainer::calculate_vehicle_linear_velocity(tf::Vector3& lin_vel, srr_msgs::TractionControlDebug& msg)
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

    msg.move_direction = curr_move_direction;
    msg.move_type = curr_move_type;
    msg.vehicle_linear_velocity.x = lin_vel.x();
    msg.vehicle_linear_velocity.y = lin_vel.y();
    msg.vehicle_linear_velocity.z = lin_vel.z();

    return rv;
}

bool SRR::TractionControlContainer::estimate_contact_angles(tf::Vector3 lin_vel, LegAbstractMap<double>& cont_ang, srr_msgs::TractionControlDebug& msg)
{
    msg.positions_window_size = leg_pivot_positions_window_size;
    // WARNING: Assumes no rotation from vehicle origin to home position leg pivot joint, ensure that's correct
    int lp_max = static_cast<int>(LegPivotEnum::RIGHT_FAR);
    for (int lp = static_cast<int>(LegPivotEnum::LEFT_NEAR), i = 0; lp <= lp_max; lp++, i++)
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

        msg.leg_pivot_position_windows[i].index = lp;
        msg.leg_pivot_position_windows[i].data = std::vector<double>(
            recent_leg_pivot_positions[lpe].begin(),
            recent_leg_pivot_positions[lpe].end()
        );
        msg.leg_pivot_position_averages[i] = alp;
        msg.trans_leg_pivot_to_wheel[i].x = trans_leg_pivot_to_wheel_frame_wrt_body.x();
        msg.trans_leg_pivot_to_wheel[i].y = trans_leg_pivot_to_wheel_frame_wrt_body.y();
        msg.trans_leg_pivot_to_wheel[i].z = trans_leg_pivot_to_wheel_frame_wrt_body.z();
        msg.wheel_linear_velocities[i] = lin_vel_wheel_wrt_body.x();
        msg.wheel_linear_velocities[i] = lin_vel_wheel_wrt_body.y();
        msg.wheel_linear_velocities[i] = lin_vel_wheel_wrt_body.z();
        msg.wheel_contact_angles[i] = cont_ang[lpe];
    }
    return true;
}

SRR::TractionControlContainer::TractionControlContainer(
    std::string _model_name,
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
    model_name(_model_name),
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

void SRR::TractionControlContainer::handle_model_states_callback(const gazebo_msgs::ModelStates& msg)
{
    for (std::size_t i = 0; i < msg.name.size(); i++)
    {
        if (model_name.compare(msg.name[i]) == 0)
        {
            if (msg.twist[i].angular.z > ANG_VEL_EPS)
            {
                curr_move_type = (msg.twist[i].angular.z > 0) ? MoveTypeEnum::TURN_LEFT : MoveTypeEnum::TURN_RIGHT;
            }
            else
            {
                curr_move_type = MoveTypeEnum::STRAIGHT;
            }

            if (SRR::mag(msg.twist[i].linear.x, msg.twist[i].linear.y, msg.twist[i].linear.z) > LIN_ACC_EPS)
            {
                //curr_move_direction = (msg.linear_acceleration.x > 0) ? MoveDirectionEnum::FORWARD : MoveDirectionEnum::BACKWARD;
                curr_move_direction = MoveDirectionEnum::FORWARD;
            }
            else
            {
                curr_move_direction = MoveDirectionEnum::STATIONARY;
            }
            break;
        }
    }
}

void SRR::TractionControlContainer::handle_imu_callback(const sensor_msgs::Imu& msg)
{
    curr_vehicle_angular_velocity.setValue(
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z
    );
}

bool SRR::TractionControlContainer::calculate_wheel_rates(LegAbstractMap<double>& contact_angles, srr_msgs::TractionControlDebug& msg)
{
    bool rv = false;
    tf::Vector3 vehicle_linear_velocity;

    if (recent_leg_pivot_positions[LegPivotEnum::LEFT_NEAR].size() != leg_pivot_positions_window_size)
    {
        goto END;
    }
    if (!calculate_vehicle_linear_velocity(vehicle_linear_velocity, msg))
    {
        goto END;
    }
    if (!estimate_contact_angles(vehicle_linear_velocity, contact_angles, msg))
    {
        goto END;
    }

    rv = true;

END:
    msg.valid = rv;
    return rv;
}
