#include "srr_kinematics/TractionControlContainer.hpp"

#include <tf/LinearMath/Matrix3x3.h>

#include "srr_utils/math_utils.hpp"
#include "srr_utils/ros_utils.hpp"

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
    //
    // This routine was too messy to try and make into a loop, so each leg is broken out and calculated separately
    //

    msg.positions_window_size = leg_pivot_positions_window_size;

    //
    // FRONT LEFT LEG
    //
    LegPivotEnum lpe = LegPivotEnum::LEFT_NEAR;

    // Calculate the translation from the body to the leg pivot frame
    tf::Vector3 trans_body_to_leg_pivot_frame = lateral_vector_origin_leg_pivot.at(lpe);

    // Calculate the rotation between the body and leg pivot frame
    double alp = -leg_pivot_position_averages[lpe];
    tf::Matrix3x3 rotation_body_leg_pivot(std::cos(alp),  0, std::sin(alp),
                                          0,              1, 0,
                                          -std::sin(alp), 0, std::cos(alp)
    );
    // Calculate the rotation between the steering and wheel frame
    double wsp = curr_wheel_steer_direction_front_left;
    tf::Matrix3x3 rotation_leg_pivot_wheel(std::cos(wsp),  0, std::sin(wsp),
                                           0,              1, 0,
                                           -std::sin(wsp), 0, std::cos(wsp)
    );
    // Calculate the rotation between the body and wheel frame
    tf::Matrix3x3 rotation_body_wheel = rotation_body_leg_pivot * rotation_leg_pivot_wheel;

    // Calculate the displacement vector between the leg pivot frame and the wheel frame resolved in the pivot and body frames
    tf::Vector3 dist_leg_pivot_to_wheel_frame_wrt_pivot(
        (leg_length * std::cos(alp)) - (wheel_connection_vertical_link_length * std::sin(alp)),
        wheel_connection_horizontal_link_length,
        -(leg_length * std::sin(alp)) - (wheel_connection_vertical_link_length * std::cos(alp))
    );
    tf::Vector3 dist_leg_pivot_to_wheel_frame_wrt_body = rotation_body_leg_pivot.inverse() * dist_leg_pivot_to_wheel_frame_wrt_pivot;

    // Calculate the angular velocity vector of the pivot joint wrt the body of the vehicle
    tf::Vector3 ang_vel_leg_pivot_frame_wrt_body(
        curr_vehicle_angular_velocity.x(),
        curr_vehicle_angular_velocity.y() + curr_leg_pivot_angular_velocity.at(lpe),
        curr_vehicle_angular_velocity.z()
    );

    // Calculate the linear velocity of the leg pivot frame with respect to the vehicle body
    tf::Vector3 lin_vel_pivot_wrt_body = lin_vel + curr_vehicle_angular_velocity.cross(trans_body_to_leg_pivot_frame);
    // Calculate the linear velocity of the wheel frame with respect to the vehicle body
    tf::Vector3 lin_vel_wheel_wrt_body = lin_vel_pivot_wrt_body - ang_vel_leg_pivot_frame_wrt_body.cross(dist_leg_pivot_to_wheel_frame_wrt_body);
    // Calculate the linear velocity of the wheel frame with respect to its own frame
    tf::Vector3 lin_vel_wheel_wrt_wheel = rotation_body_wheel * lin_vel_wheel_wrt_body;

    // Estimate the contact angle of the wheel given the velocity of the wheel in its own frame
    cont_ang[lpe] = -std::atan(lin_vel_wheel_wrt_wheel.z() / lin_vel_wheel_wrt_wheel.x());

    // Populate the debug/telemetry message
    msg.leg_pivot_position_windows[0].index = 0;
    msg.leg_pivot_position_windows[0].data = std::vector<double>(
        recent_leg_pivot_positions[lpe].begin(),
        recent_leg_pivot_positions[lpe].end()
    );
    msg.leg_pivot_position_averages[0] = alp;
    msg.wheel_steer_positions[0] = wsp;
    SRR::popROSMsg(rotation_body_leg_pivot, msg.body_leg_pivot_rotation[0]);
    SRR::popROSMsg(rotation_leg_pivot_wheel, msg.leg_pivot_wheel_steer_rotation[0]);
    SRR::popROSMsg(rotation_body_wheel, msg.body_wheel_steer_rotation[0]);
    SRR::popROSMsg(dist_leg_pivot_to_wheel_frame_wrt_body, msg.trans_leg_pivot_to_wheel_wrt_body[0]);
    SRR::popROSMsg(dist_leg_pivot_to_wheel_frame_wrt_pivot, msg.trans_leg_pivot_to_wheel_wrt_pivot[0]);
    SRR::popROSMsg(lin_vel_pivot_wrt_body, msg.pivot_linear_velocities_wrt_body[0]);
    SRR::popROSMsg(lin_vel_wheel_wrt_body, msg.wheel_linear_velocities_wrt_body[0]);
    SRR::popROSMsg(lin_vel_wheel_wrt_wheel, msg.wheel_linear_velocities_wrt_wheel[0]);
    msg.wheel_contact_angles[0] = cont_ang[lpe];

//    // WARNING: Assumes no rotation from vehicle origin to home position leg pivot joint, ensure that's correct
//    int lp_max = static_cast<int>(LegPivotEnum::RIGHT_FAR);
//    for (int lp = static_cast<int>(LegPivotEnum::LEFT_NEAR), i = 0; lp <= lp_max; lp++, i++)
//    {
//        // Get the enum for this pivot
//        LegPivotEnum lpe = static_cast<LegPivotEnum>(lp);
//
//        // Calculate the angular velocity vector of the pivot joint wrt the body of the vehicle
//        tf::Vector3 ang_vel_leg_pivot_frame_wrt_body(
//            curr_vehicle_angular_velocity.x(),
//            curr_vehicle_angular_velocity.y() + curr_leg_pivot_angular_velocity.at(lpe),
//            curr_vehicle_angular_velocity.y()
//        );
//
//        // Calculate the diplacement vector between the leg pivot frame and the wheel frame
//        double alp = leg_pivot_position_averages[lpe];
//        tf::Vector3 trans_leg_pivot_to_wheel_frame_wrt_body(
//            (leg_length * std::cos(alp)) - ((wheel_connection_vertical_link_length + wheel_radius) * std::sin(alp)),
//            0,
//            -(leg_length * std::sin(alp)) - ((wheel_connection_vertical_link_length + wheel_radius) * std::cos(alp))
//        );
//
//        // Calculate the rotation between the leg pivot frame and the wheel frame
//        tf::Matrix3x3 rotationY_origin_leg_pivot(std::cos(alp),  0, std::sin(alp),
//                                                0,              1, 0,
//                                                -std::sin(alp), 0, std::cos(alp)
//        );
//        tf::Vector3 lin_vel_wheel_wrt_body = lin_vel + curr_vehicle_angular_velocity.cross(lateral_vector_origin_leg_pivot.at(lpe))
//                                                     - ang_vel_leg_pivot_frame_wrt_body.cross(rotationY_origin_leg_pivot * trans_leg_pivot_to_wheel_frame_wrt_body);
//        cont_ang[lpe] = -std::atan(lin_vel_wheel_wrt_body.z() / lin_vel_wheel_wrt_body.x());
//
//        msg.leg_pivot_position_windows[i].index = lp;
//        msg.leg_pivot_position_windows[i].data = std::vector<double>(
//            recent_leg_pivot_positions[lpe].begin(),
//            recent_leg_pivot_positions[lpe].end()
//        );
//        msg.leg_pivot_position_averages[i] = alp;
//        msg.trans_leg_pivot_to_wheel[i].x = trans_leg_pivot_to_wheel_frame_wrt_body.x();
//        msg.trans_leg_pivot_to_wheel[i].y = trans_leg_pivot_to_wheel_frame_wrt_body.y();
//        msg.trans_leg_pivot_to_wheel[i].z = trans_leg_pivot_to_wheel_frame_wrt_body.z();
//        msg.wheel_linear_velocities[i].x = lin_vel_wheel_wrt_body.x();
//        msg.wheel_linear_velocities[i].y = lin_vel_wheel_wrt_body.y();
//        msg.wheel_linear_velocities[i].z = lin_vel_wheel_wrt_body.z();
//        msg.wheel_contact_angles[i] = cont_ang[lpe];
//    }
    return true;
}

SRR::TractionControlContainer::TractionControlContainer(
    std::string _model_name,
    double _wheel_radius,
    double _max_wheel_rate,
    double _leg_length,
    double _wheel_connection_vertical_link_length,
    std::string _leg_pivot_joint_name_left_near,
    std::string _leg_pivot_joint_name_right_near,
    std::string _leg_pivot_joint_name_left_far,
    std::string _leg_pivot_joint_name_right_far,
    double _lateral_distance_origin_leg_pivot_left_near,
    double _lateral_distance_origin_leg_pivot_right_near,
    double _lateral_distance_origin_leg_pivot_left_far,
    double _lateral_distance_origin_leg_pivot_right_far,
    std::string _wheel_joint_name_front_left,
    std::string _wheel_joint_name_front_right,
    int _leg_pivot_positions_window_size
) :
    model_name(_model_name),
    wheel_radius(_wheel_radius),
    max_wheel_rate(_max_wheel_rate),
    leg_length(_leg_length),
    wheel_connection_vertical_link_length(_wheel_connection_vertical_link_length),
    leg_pivot_positions_window_size(_leg_pivot_positions_window_size),
    wheel_joint_name_front_left(_wheel_joint_name_front_left),
    wheel_joint_name_front_right(_wheel_joint_name_front_right),
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

            double avg_position = 0.0;
            for (double pos : recent_leg_pivot_positions[lp])
            {
                avg_position += pos;
            }
            leg_pivot_position_averages[lp] = avg_position / recent_leg_pivot_positions[lp].size();

            curr_leg_pivot_angular_velocity[lp] = msg.velocity[i];
        }
        else if (name.compare(wheel_joint_name_front_left) == 0)
        {
            curr_wheel_steer_direction_front_left = msg.position[i];
        }
        else if (name.compare(wheel_joint_name_front_right) == 0)
        {
            curr_wheel_steer_direction_front_right = msg.position[i];
        }
    }

    double lpln = recent_leg_pivot_positions[LegPivotEnum::LEFT_NEAR].front();
    curr_longitudinal_distance_origin_axle = (leg_length * std::cos(lpln)) - (wheel_connection_vertical_link_length * std::sin(lpln));
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
