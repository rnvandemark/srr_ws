#ifndef __SRR_VEHICLE_VEL_KIN_CONTAINER_HPP__
#define __SRR_VEHICLE_VEL_KIN_CONTAINER_HPP__

#include "ros/ros.h"

#include "srr_msgs/VehicleVelKinDebug.h"
#include "srr_msgs/CalculateVehicleVelKin.h"

namespace SRR {

class VehicleVelKinContainer {
private:
    ros::Publisher pub_debug;               // The publisher for the debug message to publish at the end of the callback
    srr_msgs::VehicleVelKinDebug msg_debug; // The debug message to publish

    double theta_i;     // The angle of the interior wheel when making a turn
    double leg_length;  // The length of the leg of each wheel
    double conn_length; // The depth between the wheel leg and wheel axis of rotation
    double d;           // The lateral wheel separation distance
    double Rw;          // The radius of the wheels

public:
    VehicleVelKinContainer(
        ros::Publisher& _pub_debug,
        double _theta_i,
        double _leg_length,
        double _conn_length,
        double _d,
        double _Rw);

    bool handle_callback(srr_msgs::CalculateVehicleVelKin::Request&  req,
                         srr_msgs::CalculateVehicleVelKin::Response& res);
};  // class VehicleVelKinContainer

}   // namespace SRR

#endif  // __SRR_VEHICLE_VEL_KIN_CONTAINER_HPP__
