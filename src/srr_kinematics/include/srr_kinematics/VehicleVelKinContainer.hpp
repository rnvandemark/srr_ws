#ifndef __SRR_VEHICLE_VEL_KIN_CONTAINER_HPP__
#define __SRR_VEHICLE_VEL_KIN_CONTAINER_HPP__

#include "srr_msgs/CalculateVehicleVelKin.h"

namespace SRR {

class VehicleVelKinContainer {
private:
    double theta_i;     // The angle of the interior wheel when making a turn
    double leg_length;  // The length of the leg of each wheel
    double conn_length; // The depth between the wheel leg and wheel axis of rotation
    double d;           // The lateral wheel separation distance
    double Rw;          // The radius of the wheels

public:
    VehicleVelKinContainer(double _theta_i, double _leg_length, double _conn_length, double _d, double _Rw);

    bool handle_callback(srr_msgs::CalculateVehicleVelKin::Request&  req,
                         srr_msgs::CalculateVehicleVelKin::Response& res);
};  // class VehicleVelKinContainer

}   // namespace SRR

#endif  // __SRR_VEHICLE_VEL_KIN_CONTAINER_HPP__
