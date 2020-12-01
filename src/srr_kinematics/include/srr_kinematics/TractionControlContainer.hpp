#ifndef __SRR_TRACTION_CONTROL_CONTAINER_HPP__
#define __SRR_TRACTION_CONTROL_CONTAINER_HPP__

#include <tf/LinearMath/Vector3.h>

#include <srr_msgs/TractionControlDebug.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3.h>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <deque>

namespace SRR {

class TractionControlContainer {
public:
    enum MoveDirectionEnum {BACKWARD = -1, FORWARD = 1};
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

    // The name of the model in Gazebo
    const std::string model_name;
    // The radius of each wheel
    const double wheel_radius;
    // Each wheel's max angular velocity
    const double max_wheel_rate;
    // Each leg's length, from its pivot to the wheel connection link
    const double leg_length;
    // The length of the vertical link between each wheel and the end of the leg
    const double wheel_connection_vertical_link_length;
    // The length of the horizontal link between each wheel and the end of the leg
    const double wheel_connection_horizontal_link_length = 0.04445;
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
    // The moving window average of each leg pivots' position
    LegAbstractMap<double> leg_pivot_position_averages;
    // The name of the front left wheel
    std::string wheel_joint_name_front_left;
    // The name of the front right wheel
    std::string wheel_joint_name_front_right;
    // The steer position of the front left wheel
    double curr_wheel_steer_direction_front_left;
    // The steer position of the front right wheel
    double curr_wheel_steer_direction_front_right;
    // Whether or not the objective linear velocity is set
    bool curr_objective_vehicle_linear_velocity_set;
    // The objective vehicle linear velocity used to calculate the wheel speeds
    tf::Vector3 curr_objective_vehicle_linear_velocity;

    /*
     * Methods used by calculate_wheel_rates to break it up into digestible sections.
     */

    bool calculate_vehicle_linear_velocity(tf::Vector3& lin_vel, srr_msgs::TractionControlDebug& msg);
    bool estimate_contact_angles(
        tf::Vector3 lin_vel,
        LegAbstractMap<double>& cont_ang,
        srr_msgs::TractionControlDebug& msg
    );
    bool calculate_commanded_wheel_rates(
        LegAbstractMap<double> cont_ang,
        LegAbstractMap<double>& cmd_wheel_rates,
        srr_msgs::TractionControlDebug& msg
    );

public:
    TractionControlContainer(
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
        std::string _wheel_joint_name_front_left,
        std::string _wheel_joint_name_front_right,
        int _leg_pivot_positions_window_size
    );

    void handle_joint_state_callback(const sensor_msgs::JointState& msg);
    void handle_model_states_callback(const gazebo_msgs::ModelStates& msg);
    void handle_imu_callback(const sensor_msgs::Imu& msg);
    void handle_unset_velocity_command_callback(const std_msgs::Empty& msg);
    void handle_velocity_command_callback(const geometry_msgs::Vector3& msg);

    bool calculate_wheel_rates(srr_msgs::TractionControlDebug& msg);

};	// class TractionControlContainer

}	// namespace SRR

#endif	// __SRR_TRACTION_CONTROL_CONTAINER_HPP__
