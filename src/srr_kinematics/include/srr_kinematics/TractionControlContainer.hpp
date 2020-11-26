#ifndef __SRR_TRACTION_CONTROL_CONTAINER_HPP__
#define __SRR_TRACTION_CONTROL_CONTAINER_HPP__

#include <tf/LinearMath/Vector3.h>

#include <array>

namespace SRR {

class TractionControlContainer {
public:
    enum MoveDirectionEnum {BACKWARD = -1, FORWARD = 1, STATIONARY};
    enum MoveTypeEnum {STRAIGHT = 1, TURN};

protected:
    double wheel_radius;
    double max_wheel_rate;
    double longitudinal_distance_origin_front;
    double longitudinal_distance_origin_rear;
    MoveDirectionEnum curr_move_direction;
    MoveTypeEnum curr_move_type;
    double curr_dist_between_ICC_origin;
    double curr_lateral_distance_origin_front;
    double curr_lateral_distance_origin_rear;

    bool calculate_srr_linear_velocity(tf::Vector3& lin_vel);

public:
    TractionControlContainer(
        double _wheel_radius,
        double _max_wheel_rate,
        double _longitudinal_distance_origin_front,
        double _longitudinal_distance_origin_rear
    );

    bool publish_wheel_rates();

};	// class TractionControlContainer

}	// namespace SRR

#endif	// __SRR_TRACTION_CONTROL_CONTAINER_HPP__
