#ifndef __SRR_TRACTION_CONTROL_CONTAINER_HPP__
#define __SRR_TRACTION_CONTROL_CONTAINER_HPP__

#include <tf/LinearMath/Vector3.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

#include <cstddef>
#include <string>
#include <array>
#include <unordered_map>
#include <deque>

namespace SRR {

class TractionControlContainer {
public:
    enum MoveDirectionEnum {BACKWARD = -1, FORWARD = 1, STATIONARY};
    enum MoveTypeEnum {STRAIGHT = 4, TURN_LEFT, TURN_RIGHT};
    enum LegPivotEnum {LEFT_NEAR = 8, RIGHT_NEAR, LEFT_FAR, RIGHT_FAR};
    struct LegPivotEnumHash
    {
        template <typename T>
        std::size_t operator()(T t) const
        {
            return static_cast<std::size_t>(t);
        }
    };

    template <typename T>
    using LegAbstractMap = std::unordered_map<LegPivotEnum, T, LegPivotEnumHash>;
    template <typename T>
    using AbstractLegMap = std::unordered_map<T, LegPivotEnum>;

protected:
    /*
     * Magic threshold numbers
     */
    static constexpr double LIN_ACC_EPS = 0.05;
    static constexpr double ANG_VEL_EPS = 0.05;

    /*
     * Configuration parameters that are initialized on startup and then final.
     */
    // The radius of each wheel
    const double wheel_radius;
    // Each wheel's max angular velocity
    const double max_wheel_rate;
    // Each leg's length, from its pivot to the wheel connection link
    const double leg_length;
    // The length of the link between each wheel and the end of the leg
    const double wheel_connection_link_length;
    // Names of the joints that allow for rotation about the leg pivot
    const AbstractLegMap<std::string> leg_pivot_joint_names;
    // Lateral dist between the vehicle origin and the center of the leg pivot for each wheel
    const LegAbstractMap<tf::Vector3> lateral_vector_origin_leg_pivot;
    // The size of the window used to calculate the moving average
    const std::size_t leg_pivot_positions_window_size;

    /*
     * Inputs that are updated on receiving messages.
     * WARNING: for now, the following assumptions concerning some/all of these values are made:
     *  - they are updated consistently
     *  - the left and right leg pivot position pairs are equal, so the longitudinal distance is easy to find
     *  - curr_dist_between_ICC_origin is just a constant for now
     */

    // The direction the vehicle is currently moving
    MoveDirectionEnum curr_move_direction;
    // The type of movement (straight or turn) the vehicle is currently doing
    MoveTypeEnum curr_move_type;
    // Lateral dist between the vehicle origin and the instantaneous center of curvature (for turns)
    double curr_dist_between_ICC_origin;
    // Longitudinal dist between the vehicle origin and each imaginary axle
    double curr_longitudinal_distance_origin_axle;
    // The angular velocity of the chassis about the origin
    tf::Vector3 curr_vehicle_angular_velocity;
    // The instanenous angular velocity of the leg pivot for each wheel
    LegAbstractMap<double> curr_leg_pivot_angular_velocity;
    // The positions for the leg pivots used to calculate the moving window average
    LegAbstractMap<std::deque<double>> recent_leg_pivot_positions;

    /*
     * Methods used by publish_wheel_rates to break it up into digestible sections.
     */

    bool calculate_vehicle_linear_velocity(tf::Vector3& lin_vel);
    bool estimate_contact_angles(tf::Vector3 lin_vel, LegAbstractMap<double>& cont_ang);

public:
    TractionControlContainer(
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
    );

    void handle_joint_state_callback(const sensor_msgs::JointState& msg);
    void handle_imu_callback(const sensor_msgs::Imu& msg);

    bool publish_wheel_rates(LegAbstractMap<double>& contact_angles);

};	// class TractionControlContainer

}	// namespace SRR

#endif	// __SRR_TRACTION_CONTROL_CONTAINER_HPP__
