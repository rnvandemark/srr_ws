#ifndef __SRR_TRACTION_CONTROL_CONTAINER_HPP__
#define __SRR_TRACTION_CONTROL_CONTAINER_HPP__

#include <tf/LinearMath/Vector3.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

#include <cstddef>
#include <string>
#include <array>
#include <unordered_map>

namespace SRR {

class TractionControlContainer {
public:
    enum MoveDirectionEnum {BACKWARD = -1, FORWARD = 1, STATIONARY};
    enum MoveTypeEnum {STRAIGHT = 4, TURN_LEFT, TURN_RIGHT};
    enum BogiePivotEnum {LEFT_NEAR = 8, RIGHT_NEAR, LEFT_FAR, RIGHT_FAR};
    struct BogiePivotEnumHash
    {
        template <typename T>
        std::size_t operator()(T t) const
        {
            return static_cast<std::size_t>(t);
        }
    };

    template <typename T>
    using BogieAbstractMap = std::unordered_map<BogiePivotEnum, T, BogiePivotEnumHash>;
    template <typename T>
    using AbstractBogieMap = std::unordered_map<T, BogiePivotEnum>;

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
    // Each leg's length, from its bogie pivot to the wheel connection link
    const double bogie_leg_length;
    // The length of the link between each wheel and the distal end of the bogie leg
    const double wheel_connection_link_length;
    // Names of the joints that allow for rotation about the bogie pivot
    const AbstractBogieMap<std::string> bogie_pivot_joint_names;
    // Lateral dist between the vehicle origin and the center of the bogie pivot for each wheel
    const BogieAbstractMap<tf::Vector3> lateral_vector_origin_bogie_pivot;

    /*
     * Inputs that are updated on receiving messages.
     * WARNING: for now, the following assumptions concerning some/all of these values are made:
     *  - they are updated consistently
     *  - the left and right bogie position pairs are equal, so the longitudinal distance is easy to find
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
    // The current position of each leg off the bogie pivots
    BogieAbstractMap<double> curr_bogie_positions;
    // The instanenous angular velocity of the bogie pivot for each wheel
    BogieAbstractMap<double> curr_bogie_angular_velocity;

    /*
     * Methods used by publish_wheel_rates to break it up into digestible sections.
     */

    bool calculate_vehicle_linear_velocity(tf::Vector3& lin_vel);
    bool estimate_contact_angles(tf::Vector3 lin_vel, BogieAbstractMap<double>& cont_ang);

public:
    TractionControlContainer(
        double _wheel_radius,
        double _max_wheel_rate,
        double _bogie_leg_length,
        double _wheel_connection_link_length,
        std::string bogie_pivot_joint_name_left_near,
        std::string bogie_pivot_joint_name_right_near,
        std::string bogie_pivot_joint_name_left_far,
        std::string bogie_pivot_joint_name_right_far,
        double lateral_distance_origin_bogie_pivot_left_near,
        double lateral_distance_origin_bogie_pivot_right_near,
        double lateral_distance_origin_bogie_pivot_left_far,
        double lateral_distance_origin_bogie_pivot_right_far
    );

    void handle_joint_state_callback(const sensor_msgs::JointState& msg);
    void handle_imu_callback(const sensor_msgs::Imu& msg);

    bool publish_wheel_rates(BogieAbstractMap<double>& contact_angles);

};	// class TractionControlContainer

}	// namespace SRR

#endif	// __SRR_TRACTION_CONTROL_CONTAINER_HPP__
