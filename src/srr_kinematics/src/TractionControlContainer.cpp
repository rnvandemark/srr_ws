#include "srr_kinematics/TractionControlContainer.hpp"

#include "srr_utils/math_utils.hpp"
#include "srr_utils/ros_utils.hpp"

#include <cmath>
#include <algorithm>

bool SRR::TractionControlContainer::calculate_vehicle_linear_velocity(tf::Vector3& lin_vel, srr_msgs::TractionControlDebug& msg)
{
    bool rv = false;
    double x_dot = 0.0;

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

    lin_vel.setValue(x_dot, 0, 0);

    msg.move_direction = curr_move_direction;
    msg.move_type = curr_move_type;
    SRR::popROSMsg(lin_vel, msg.vehicle_linear_velocity);

    return rv;
}

bool SRR::TractionControlContainer::estimate_contact_angles(
    tf::Vector3 lin_vel,
    LegAbstractMap<double>& cont_ang,
    srr_msgs::TractionControlDebug& msg
)
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
        curr_vehicle_angular_velocity.y() - curr_leg_pivot_angular_velocity.at(lpe),
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

    //
    // FRONT RIGHT LEG
    // Start reusing the variables declared above
    //
    lpe = LegPivotEnum::RIGHT_NEAR;

    // Calculate the translation from the body to the leg pivot frame
    trans_body_to_leg_pivot_frame = lateral_vector_origin_leg_pivot.at(lpe);

    // Calculate the rotation between the body and leg pivot frame
    alp = -leg_pivot_position_averages[lpe];
    rotation_body_leg_pivot.setValue(std::cos(alp),  0, std::sin(alp),
                                     0,              1, 0,
                                     -std::sin(alp), 0, std::cos(alp)
    );
    // Calculate the rotation between the steering and wheel frame
    wsp = curr_wheel_steer_direction_front_right;
    rotation_leg_pivot_wheel.setValue(std::cos(wsp),  0, std::sin(wsp),
                                      0,              1, 0,
                                      -std::sin(wsp), 0, std::cos(wsp)
    );
    // Calculate the rotation between the body and wheel frame
    rotation_body_wheel = rotation_body_leg_pivot * rotation_leg_pivot_wheel;

    // Calculate the displacement vector between the leg pivot frame and the wheel frame resolved in the pivot and body frames
    dist_leg_pivot_to_wheel_frame_wrt_pivot.setValue(
        (leg_length * std::cos(alp)) - (wheel_connection_vertical_link_length * std::sin(alp)),
        -wheel_connection_horizontal_link_length,
        -(leg_length * std::sin(alp)) - (wheel_connection_vertical_link_length * std::cos(alp))
    );
    dist_leg_pivot_to_wheel_frame_wrt_body = rotation_body_leg_pivot.inverse() * dist_leg_pivot_to_wheel_frame_wrt_pivot;

    // Calculate the angular velocity vector of the pivot joint wrt the body of the vehicle
    ang_vel_leg_pivot_frame_wrt_body.setValue(
        curr_vehicle_angular_velocity.x(),
        curr_vehicle_angular_velocity.y() - curr_leg_pivot_angular_velocity.at(lpe),
        curr_vehicle_angular_velocity.z()
    );

    // Calculate the linear velocity of the leg pivot frame with respect to the vehicle body
    lin_vel_pivot_wrt_body = lin_vel + curr_vehicle_angular_velocity.cross(trans_body_to_leg_pivot_frame);
    // Calculate the linear velocity of the wheel frame with respect to the vehicle body
    lin_vel_wheel_wrt_body = lin_vel_pivot_wrt_body - ang_vel_leg_pivot_frame_wrt_body.cross(dist_leg_pivot_to_wheel_frame_wrt_body);
    // Calculate the linear velocity of the wheel frame with respect to its own frame
    lin_vel_wheel_wrt_wheel = rotation_body_wheel * lin_vel_wheel_wrt_body;

    // Estimate the contact angle of the wheel given the velocity of the wheel in its own frame
    cont_ang[lpe] = -std::atan(lin_vel_wheel_wrt_wheel.z() / lin_vel_wheel_wrt_wheel.x());

    // Populate the debug/telemetry message
    msg.leg_pivot_position_windows[1].index = 1;
    msg.leg_pivot_position_windows[1].data = std::vector<double>(
        recent_leg_pivot_positions[lpe].begin(),
        recent_leg_pivot_positions[lpe].end()
    );
    msg.leg_pivot_position_averages[1] = alp;
    msg.wheel_steer_positions[1] = wsp;
    SRR::popROSMsg(rotation_body_leg_pivot, msg.body_leg_pivot_rotation[1]);
    SRR::popROSMsg(rotation_leg_pivot_wheel, msg.leg_pivot_wheel_steer_rotation[1]);
    SRR::popROSMsg(rotation_body_wheel, msg.body_wheel_steer_rotation[1]);
    SRR::popROSMsg(dist_leg_pivot_to_wheel_frame_wrt_body, msg.trans_leg_pivot_to_wheel_wrt_body[1]);
    SRR::popROSMsg(dist_leg_pivot_to_wheel_frame_wrt_pivot, msg.trans_leg_pivot_to_wheel_wrt_pivot[1]);
    SRR::popROSMsg(lin_vel_pivot_wrt_body, msg.pivot_linear_velocities_wrt_body[1]);
    SRR::popROSMsg(lin_vel_wheel_wrt_body, msg.wheel_linear_velocities_wrt_body[1]);
    SRR::popROSMsg(lin_vel_wheel_wrt_wheel, msg.wheel_linear_velocities_wrt_wheel[1]);
    msg.wheel_contact_angles[1] = cont_ang[lpe];

    //
    // BACK LEFT LEG
    //
    lpe = LegPivotEnum::LEFT_FAR;

    // Calculate the translation from the body to the leg pivot frame
    trans_body_to_leg_pivot_frame = lateral_vector_origin_leg_pivot.at(lpe);

    // Calculate the rotation between the body and leg pivot frame
    alp = -leg_pivot_position_averages[lpe];
    rotation_body_leg_pivot.setValue(std::cos(alp),  0, std::sin(alp),
                                     0,              1, 0,
                                     -std::sin(alp), 0, std::cos(alp)
    );
    // Set the static rotation between the steering and wheel frame (reset wsp for telemetry)
    wsp = 0;
    rotation_leg_pivot_wheel.setValue(1, 0, 0,
                                      0, 1, 0,
                                      0, 0, 1
    );
    // Calculate the rotation between the body and wheel frame
    rotation_body_wheel = rotation_body_leg_pivot * rotation_leg_pivot_wheel;

    // Calculate the displacement vector between the leg pivot frame and the wheel frame resolved in the pivot and body frames
    dist_leg_pivot_to_wheel_frame_wrt_pivot.setValue(
        -(leg_length * std::cos(alp)) + (wheel_connection_vertical_link_length * std::sin(alp)),
        wheel_connection_horizontal_link_length,
        -(leg_length * std::sin(alp)) - (wheel_connection_vertical_link_length * std::cos(alp))
    );
    dist_leg_pivot_to_wheel_frame_wrt_body = rotation_body_leg_pivot.inverse() * dist_leg_pivot_to_wheel_frame_wrt_pivot;

    // Calculate the angular velocity vector of the pivot joint wrt the body of the vehicle
    ang_vel_leg_pivot_frame_wrt_body.setValue(
        curr_vehicle_angular_velocity.x(),
        curr_vehicle_angular_velocity.y() + curr_leg_pivot_angular_velocity.at(lpe),
        curr_vehicle_angular_velocity.z()
    );

    // Calculate the linear velocity of the leg pivot frame with respect to the vehicle body
    lin_vel_pivot_wrt_body = lin_vel + curr_vehicle_angular_velocity.cross(trans_body_to_leg_pivot_frame);
    // Calculate the linear velocity of the wheel frame with respect to the vehicle body
    lin_vel_wheel_wrt_body = lin_vel_pivot_wrt_body - ang_vel_leg_pivot_frame_wrt_body.cross(dist_leg_pivot_to_wheel_frame_wrt_body);
    // Calculate the linear velocity of the wheel frame with respect to its own frame
    lin_vel_wheel_wrt_wheel = rotation_body_wheel * lin_vel_wheel_wrt_body;

    // Estimate the contact angle of the wheel given the velocity of the wheel in its own frame
    cont_ang[lpe] = -std::atan(lin_vel_wheel_wrt_wheel.z() / lin_vel_wheel_wrt_wheel.x());

    // Populate the debug/telemetry message
    msg.leg_pivot_position_windows[2].index = 2;
    msg.leg_pivot_position_windows[2].data = std::vector<double>(
        recent_leg_pivot_positions[lpe].begin(),
        recent_leg_pivot_positions[lpe].end()
    );
    msg.leg_pivot_position_averages[2] = alp;
    msg.wheel_steer_positions[2] = wsp;
    SRR::popROSMsg(rotation_body_leg_pivot, msg.body_leg_pivot_rotation[2]);
    SRR::popROSMsg(rotation_leg_pivot_wheel, msg.leg_pivot_wheel_steer_rotation[2]);
    SRR::popROSMsg(rotation_body_wheel, msg.body_wheel_steer_rotation[2]);
    SRR::popROSMsg(dist_leg_pivot_to_wheel_frame_wrt_body, msg.trans_leg_pivot_to_wheel_wrt_body[2]);
    SRR::popROSMsg(dist_leg_pivot_to_wheel_frame_wrt_pivot, msg.trans_leg_pivot_to_wheel_wrt_pivot[2]);
    SRR::popROSMsg(lin_vel_pivot_wrt_body, msg.pivot_linear_velocities_wrt_body[2]);
    SRR::popROSMsg(lin_vel_wheel_wrt_body, msg.wheel_linear_velocities_wrt_body[2]);
    SRR::popROSMsg(lin_vel_wheel_wrt_wheel, msg.wheel_linear_velocities_wrt_wheel[2]);
    msg.wheel_contact_angles[2] = cont_ang[lpe];

    //
    // BACK RIGHT LEG
    //
    lpe = LegPivotEnum::RIGHT_FAR;

    // Calculate the translation from the body to the leg pivot frame
    trans_body_to_leg_pivot_frame = lateral_vector_origin_leg_pivot.at(lpe);

    // Calculate the rotation between the body and leg pivot frame
    alp = -leg_pivot_position_averages[lpe];
    rotation_body_leg_pivot.setValue(std::cos(alp),  0, std::sin(alp),
                                     0,              1, 0,
                                     -std::sin(alp), 0, std::cos(alp)
    );
    // Set the static rotation between the steering and wheel frame (reset wsp for telemetry)
    wsp = 0;
    rotation_leg_pivot_wheel.setValue(1, 0, 0,
                                      0, 1, 0,
                                      0, 0, 1
    );
    // Calculate the rotation between the body and wheel frame
    rotation_body_wheel = rotation_body_leg_pivot * rotation_leg_pivot_wheel;

    // Calculate the displacement vector between the leg pivot frame and the wheel frame resolved in the pivot and body frames
    dist_leg_pivot_to_wheel_frame_wrt_pivot.setValue(
        -(leg_length * std::cos(alp)) + (wheel_connection_vertical_link_length * std::sin(alp)),
        -wheel_connection_horizontal_link_length,
        -(leg_length * std::sin(alp)) - (wheel_connection_vertical_link_length * std::cos(alp))
    );
    dist_leg_pivot_to_wheel_frame_wrt_body = rotation_body_leg_pivot.inverse() * dist_leg_pivot_to_wheel_frame_wrt_pivot;

    // Calculate the angular velocity vector of the pivot joint wrt the body of the vehicle
    ang_vel_leg_pivot_frame_wrt_body.setValue(
        curr_vehicle_angular_velocity.x(),
        curr_vehicle_angular_velocity.y() + curr_leg_pivot_angular_velocity.at(lpe),
        curr_vehicle_angular_velocity.z()
    );

    // Calculate the linear velocity of the leg pivot frame with respect to the vehicle body
    lin_vel_pivot_wrt_body = lin_vel + curr_vehicle_angular_velocity.cross(trans_body_to_leg_pivot_frame);
    // Calculate the linear velocity of the wheel frame with respect to the vehicle body
    lin_vel_wheel_wrt_body = lin_vel_pivot_wrt_body - ang_vel_leg_pivot_frame_wrt_body.cross(dist_leg_pivot_to_wheel_frame_wrt_body);
    // Calculate the linear velocity of the wheel frame with respect to its own frame
    lin_vel_wheel_wrt_wheel = rotation_body_wheel * lin_vel_wheel_wrt_body;

    // Estimate the contact angle of the wheel given the velocity of the wheel in its own frame
    cont_ang[lpe] = -std::atan(lin_vel_wheel_wrt_wheel.z() / lin_vel_wheel_wrt_wheel.x());

    // Populate the debug/telemetry message
    msg.leg_pivot_position_windows[3].index = 3;
    msg.leg_pivot_position_windows[3].data = std::vector<double>(
        recent_leg_pivot_positions[lpe].begin(),
        recent_leg_pivot_positions[lpe].end()
    );
    msg.leg_pivot_position_averages[3] = alp;
    msg.wheel_steer_positions[3] = wsp;
    SRR::popROSMsg(rotation_body_leg_pivot, msg.body_leg_pivot_rotation[3]);
    SRR::popROSMsg(rotation_leg_pivot_wheel, msg.leg_pivot_wheel_steer_rotation[3]);
    SRR::popROSMsg(rotation_body_wheel, msg.body_wheel_steer_rotation[3]);
    SRR::popROSMsg(dist_leg_pivot_to_wheel_frame_wrt_body, msg.trans_leg_pivot_to_wheel_wrt_body[3]);
    SRR::popROSMsg(dist_leg_pivot_to_wheel_frame_wrt_pivot, msg.trans_leg_pivot_to_wheel_wrt_pivot[3]);
    SRR::popROSMsg(lin_vel_pivot_wrt_body, msg.pivot_linear_velocities_wrt_body[3]);
    SRR::popROSMsg(lin_vel_wheel_wrt_body, msg.wheel_linear_velocities_wrt_body[3]);
    SRR::popROSMsg(lin_vel_wheel_wrt_wheel, msg.wheel_linear_velocities_wrt_wheel[3]);
    msg.wheel_contact_angles[3] = cont_ang[lpe];

    return true;
}

bool SRR::TractionControlContainer::calculate_commanded_wheel_rates(
    LegAbstractMap<double> cont_ang,
    LegAbstractMap<double>& cmd_wheel_rates,
    srr_msgs::TractionControlDebug& msg)
{
    //
    // This routine was too messy to try and make into a loop, so each leg is broken out and calculated separately
    //

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
    // Calculate the rotation between the wheel and its contact frame
    double ca = cont_ang[lpe];
    tf::Matrix3x3 rotation_wheel_contact(std::cos(ca),  0, std::sin(ca),
                                         0,             1, 0,
                                         -std::sin(ca), 0, std::cos(ca)
    );
    // Calculate the net rotation between the body and this wheel's contact frame
    tf::Matrix3x3 rotation_body_contact = rotation_wheel_contact * rotation_leg_pivot_wheel * rotation_body_leg_pivot;

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
        curr_vehicle_angular_velocity.y() - curr_leg_pivot_angular_velocity.at(lpe),
        curr_vehicle_angular_velocity.z()
    );

    // Calculate the linear velocity of the leg pivot frame with respect to the vehicle body
    tf::Vector3 lin_vel_pivot_wrt_body = curr_objective_vehicle_linear_velocity + curr_vehicle_angular_velocity.cross(trans_body_to_leg_pivot_frame);
    // Calculate the linear velocity of the wheel frame with respect to the vehicle body
    tf::Vector3 lin_vel_wheel_wrt_body = lin_vel_pivot_wrt_body - ang_vel_leg_pivot_frame_wrt_body.cross(dist_leg_pivot_to_wheel_frame_wrt_body);
    // Calculate the linear velocity of the wheel's contact frame with respect to the wheel
    tf::Vector3 lin_vel_contact_wrt_wheel = rotation_body_contact * lin_vel_wheel_wrt_body;
    // Calculate the angular velocity of the body with respect to the wheel's contact frame
    tf::Vector3 ang_vel_contact_frame = rotation_body_contact * ang_vel_leg_pivot_frame_wrt_body;

    // Finally, calculate the commanded wheel rate
    cmd_wheel_rates[lpe] = (lin_vel_contact_wrt_wheel.x() / wheel_radius) - ang_vel_contact_frame.y();

    // Populate the debug/telemetry message
    SRR::popROSMsg(lin_vel_contact_wrt_wheel, msg.objective_wheel_linear_velocities_wrt_contact[0]);
    msg.wheel_commanded_rates[0] = cmd_wheel_rates[lpe];

    //
    // FRONT RIGHT LEG
    // Start reusing the variables declared above
    //
    lpe = LegPivotEnum::RIGHT_NEAR;

    // Calculate the translation from the body to the leg pivot frame
    trans_body_to_leg_pivot_frame = lateral_vector_origin_leg_pivot.at(lpe);

    // Calculate the rotation between the body and leg pivot frame
    alp = -leg_pivot_position_averages[lpe];
    rotation_body_leg_pivot.setValue(std::cos(alp),  0, std::sin(alp),
                                     0,              1, 0,
                                     -std::sin(alp), 0, std::cos(alp)
    );
    // Calculate the rotation between the steering and wheel frame
    wsp = curr_wheel_steer_direction_front_left;
    rotation_leg_pivot_wheel.setValue(std::cos(wsp),  0, std::sin(wsp),
                                      0,              1, 0,
                                      -std::sin(wsp), 0, std::cos(wsp)
    );
    // Calculate the rotation between the wheel and its contact frame
    ca = cont_ang[lpe];
    rotation_wheel_contact.setValue(std::cos(ca),  0, std::sin(ca),
                                    0,             1, 0,
                                    -std::sin(ca), 0, std::cos(ca)
    );
    // Calculate the net rotation between the body and this wheel's contact frame
    rotation_body_contact = rotation_wheel_contact * rotation_leg_pivot_wheel * rotation_body_leg_pivot;

    // Calculate the displacement vector between the leg pivot frame and the wheel frame resolved in the pivot and body frames
    dist_leg_pivot_to_wheel_frame_wrt_pivot.setValue(
        (leg_length * std::cos(alp)) - (wheel_connection_vertical_link_length * std::sin(alp)),
        -wheel_connection_horizontal_link_length,
        -(leg_length * std::sin(alp)) - (wheel_connection_vertical_link_length * std::cos(alp))
    );
    dist_leg_pivot_to_wheel_frame_wrt_body = rotation_body_leg_pivot.inverse() * dist_leg_pivot_to_wheel_frame_wrt_pivot;

    // Calculate the angular velocity vector of the pivot joint wrt the body of the vehicle
    ang_vel_leg_pivot_frame_wrt_body.setValue(
        curr_vehicle_angular_velocity.x(),
        curr_vehicle_angular_velocity.y() - curr_leg_pivot_angular_velocity.at(lpe),
        curr_vehicle_angular_velocity.z()
    );

    // Calculate the linear velocity of the leg pivot frame with respect to the vehicle body
    lin_vel_pivot_wrt_body = curr_objective_vehicle_linear_velocity + curr_vehicle_angular_velocity.cross(trans_body_to_leg_pivot_frame);
    // Calculate the linear velocity of the wheel frame with respect to the vehicle body
    lin_vel_wheel_wrt_body = lin_vel_pivot_wrt_body - ang_vel_leg_pivot_frame_wrt_body.cross(dist_leg_pivot_to_wheel_frame_wrt_body);
    // Calculate the linear velocity of the wheel's contact frame with respect to the wheel
    lin_vel_contact_wrt_wheel = rotation_body_contact * lin_vel_wheel_wrt_body;
    // Calculate the angular velocity of the body with respect to the wheel's contact frame
    ang_vel_contact_frame = rotation_body_contact * ang_vel_leg_pivot_frame_wrt_body;

    // Finally, calculate the commanded wheel rate
    cmd_wheel_rates[lpe] = (lin_vel_contact_wrt_wheel.x() / wheel_radius) - ang_vel_contact_frame.y();

    // Populate the debug/telemetry message
    SRR::popROSMsg(lin_vel_contact_wrt_wheel, msg.objective_wheel_linear_velocities_wrt_contact[1]);
    msg.wheel_commanded_rates[1] = cmd_wheel_rates[lpe];

    //
    // BACK LEFT LEG
    //
    lpe = LegPivotEnum::LEFT_FAR;

    // Calculate the translation from the body to the leg pivot frame
    trans_body_to_leg_pivot_frame = lateral_vector_origin_leg_pivot.at(lpe);

    // Calculate the rotation between the body and leg pivot frame
    alp = -leg_pivot_position_averages[lpe];
    rotation_body_leg_pivot.setValue(std::cos(alp),  0, std::sin(alp),
                                     0,              1, 0,
                                     -std::sin(alp), 0, std::cos(alp)
    );
    // Calculate the rotation between the steering and wheel frame
    wsp = curr_wheel_steer_direction_front_left;
    rotation_leg_pivot_wheel.setValue(std::cos(wsp),  0, std::sin(wsp),
                                      0,              1, 0,
                                      -std::sin(wsp), 0, std::cos(wsp)
    );
    // Calculate the rotation between the wheel and its contact frame
    ca = cont_ang[lpe];
    rotation_wheel_contact.setValue(std::cos(ca),  0, std::sin(ca),
                                    0,             1, 0,
                                    -std::sin(ca), 0, std::cos(ca)
    );
    // Calculate the net rotation between the body and this wheel's contact frame
    rotation_body_contact = rotation_wheel_contact * rotation_leg_pivot_wheel * rotation_body_leg_pivot;

    // Calculate the displacement vector between the leg pivot frame and the wheel frame resolved in the pivot and body frames
    dist_leg_pivot_to_wheel_frame_wrt_pivot.setValue(
        -(leg_length * std::cos(alp)) + (wheel_connection_vertical_link_length * std::sin(alp)),
        wheel_connection_horizontal_link_length,
        -(leg_length * std::sin(alp)) - (wheel_connection_vertical_link_length * std::cos(alp))
    );
    dist_leg_pivot_to_wheel_frame_wrt_body = rotation_body_leg_pivot.inverse() * dist_leg_pivot_to_wheel_frame_wrt_pivot;

    // Calculate the angular velocity vector of the pivot joint wrt the body of the vehicle
    ang_vel_leg_pivot_frame_wrt_body.setValue(
        curr_vehicle_angular_velocity.x(),
        curr_vehicle_angular_velocity.y() + curr_leg_pivot_angular_velocity.at(lpe),
        curr_vehicle_angular_velocity.z()
    );

    // Calculate the linear velocity of the leg pivot frame with respect to the vehicle body
    lin_vel_pivot_wrt_body = curr_objective_vehicle_linear_velocity + curr_vehicle_angular_velocity.cross(trans_body_to_leg_pivot_frame);
    // Calculate the linear velocity of the wheel frame with respect to the vehicle body
    lin_vel_wheel_wrt_body = lin_vel_pivot_wrt_body - ang_vel_leg_pivot_frame_wrt_body.cross(dist_leg_pivot_to_wheel_frame_wrt_body);
    // Calculate the linear velocity of the wheel's contact frame with respect to the wheel
    lin_vel_contact_wrt_wheel = rotation_body_contact * lin_vel_wheel_wrt_body;
    // Calculate the angular velocity of the body with respect to the wheel's contact frame
    ang_vel_contact_frame = rotation_body_contact * ang_vel_leg_pivot_frame_wrt_body;

    // Finally, calculate the commanded wheel rate
    cmd_wheel_rates[lpe] = (lin_vel_contact_wrt_wheel.x() / wheel_radius) - ang_vel_contact_frame.y();

    // Populate the debug/telemetry message
    SRR::popROSMsg(lin_vel_contact_wrt_wheel, msg.objective_wheel_linear_velocities_wrt_contact[2]);
    msg.wheel_commanded_rates[2] = cmd_wheel_rates[lpe];

    //
    // BACK RIGHT LEG
    //
    lpe = LegPivotEnum::RIGHT_FAR;

    // Calculate the translation from the body to the leg pivot frame
    trans_body_to_leg_pivot_frame = lateral_vector_origin_leg_pivot.at(lpe);

    // Calculate the rotation between the body and leg pivot frame
    alp = -leg_pivot_position_averages[lpe];
    rotation_body_leg_pivot.setValue(std::cos(alp),  0, std::sin(alp),
                                     0,              1, 0,
                                     -std::sin(alp), 0, std::cos(alp)
    );
    // Calculate the rotation between the steering and wheel frame
    wsp = curr_wheel_steer_direction_front_left;
    rotation_leg_pivot_wheel.setValue(std::cos(wsp),  0, std::sin(wsp),
                                      0,              1, 0,
                                      -std::sin(wsp), 0, std::cos(wsp)
    );
    // Calculate the rotation between the wheel and its contact frame
    ca = cont_ang[lpe];
    rotation_wheel_contact.setValue(std::cos(ca),  0, std::sin(ca),
                                    0,             1, 0,
                                    -std::sin(ca), 0, std::cos(ca)
    );
    // Calculate the net rotation between the body and this wheel's contact frame
    rotation_body_contact = rotation_wheel_contact * rotation_leg_pivot_wheel * rotation_body_leg_pivot;

    // Calculate the displacement vector between the leg pivot frame and the wheel frame resolved in the pivot and body frames
    dist_leg_pivot_to_wheel_frame_wrt_pivot.setValue(
        -(leg_length * std::cos(alp)) + (wheel_connection_vertical_link_length * std::sin(alp)),
        -wheel_connection_horizontal_link_length,
        -(leg_length * std::sin(alp)) - (wheel_connection_vertical_link_length * std::cos(alp))
    );
    dist_leg_pivot_to_wheel_frame_wrt_body = rotation_body_leg_pivot.inverse() * dist_leg_pivot_to_wheel_frame_wrt_pivot;

    // Calculate the angular velocity vector of the pivot joint wrt the body of the vehicle
    ang_vel_leg_pivot_frame_wrt_body.setValue(
        curr_vehicle_angular_velocity.x(),
        curr_vehicle_angular_velocity.y() + curr_leg_pivot_angular_velocity.at(lpe),
        curr_vehicle_angular_velocity.z()
    );

    // Calculate the linear velocity of the leg pivot frame with respect to the vehicle body
    lin_vel_pivot_wrt_body = curr_objective_vehicle_linear_velocity + curr_vehicle_angular_velocity.cross(trans_body_to_leg_pivot_frame);
    // Calculate the linear velocity of the wheel frame with respect to the vehicle body
    lin_vel_wheel_wrt_body = lin_vel_pivot_wrt_body - ang_vel_leg_pivot_frame_wrt_body.cross(dist_leg_pivot_to_wheel_frame_wrt_body);
    // Calculate the linear velocity of the wheel's contact frame with respect to the wheel
    lin_vel_contact_wrt_wheel = rotation_body_contact * lin_vel_wheel_wrt_body;
    // Calculate the angular velocity of the body with respect to the wheel's contact frame
    ang_vel_contact_frame = rotation_body_contact * ang_vel_leg_pivot_frame_wrt_body;

    // Finally, calculate the commanded wheel rate
    cmd_wheel_rates[lpe] = (lin_vel_contact_wrt_wheel.x() / wheel_radius) - ang_vel_contact_frame.y();

    // Populate the debug/telemetry message
    SRR::popROSMsg(lin_vel_contact_wrt_wheel, msg.objective_wheel_linear_velocities_wrt_contact[2]);
    msg.wheel_commanded_rates[3] = cmd_wheel_rates[lpe];

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
    curr_objective_vehicle_linear_velocity_set(false),
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

void SRR::TractionControlContainer::handle_unset_velocity_command_callback(const std_msgs::Empty& msg)
{
    (void)msg;
    curr_objective_vehicle_linear_velocity.setValue(0, 0, 0);
    curr_objective_vehicle_linear_velocity_set = false;
}

void SRR::TractionControlContainer::handle_velocity_command_callback(const geometry_msgs::Vector3& msg)
{
    SRR::extractROSMsg(msg, curr_objective_vehicle_linear_velocity);
    curr_objective_vehicle_linear_velocity.setY(0);
    curr_objective_vehicle_linear_velocity_set = true;
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

//            if (SRR::mag(msg.twist[i].linear.x, msg.twist[i].linear.y, msg.twist[i].linear.z) > LIN_ACC_EPS)
//            {
//                //curr_move_direction = (msg.linear_acceleration.x > 0) ? MoveDirectionEnum::FORWARD : MoveDirectionEnum::BACKWARD;
//            }
            curr_move_direction = MoveDirectionEnum::FORWARD;
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

bool SRR::TractionControlContainer::calculate_wheel_rates(srr_msgs::TractionControlDebug& msg)
{
    bool rv = false;
    tf::Vector3 vehicle_linear_velocity;
    SRR::TractionControlContainer::LegAbstractMap<double> contact_angles;
    SRR::TractionControlContainer::LegAbstractMap<double> commanded_wheel_rates;

    if (!curr_objective_vehicle_linear_velocity_set)
    {
        goto END;
    }
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
    if (!calculate_commanded_wheel_rates(contact_angles, commanded_wheel_rates, msg))
    {
        goto END;
    }

    rv = true;

END:
    msg.valid = rv;
    return rv;
}
