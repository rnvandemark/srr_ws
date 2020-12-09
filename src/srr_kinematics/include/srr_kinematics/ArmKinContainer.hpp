#ifndef __SRR_ARM_KIN_CONTAINER_HPP__
#define __SRR_ARM_KIN_CONTAINER_HPP__

#include "srr_msgs/CalculateArmFwdKin.h"
#include "srr_msgs/CalculateArmInvKin.h"

#include <string>
#include <unordered_map>

namespace SRR {

class ArmKinContainer {
protected:
    std::unordered_map<std::string, double> dh;

    const double leniency;

    double s(double rad);

    double c(double rad);

    double at2(double y, double x);

    void mm(double min, double max, double& minFixed, double& maxFixed);

public:
    ArmKinContainer(double _leniency=0.000001);

    ArmKinContainer& add_param(std::string param, double value);

    bool handle_fwd_kin_callback(srr_msgs::CalculateArmFwdKin::Request&  req,
                                 srr_msgs::CalculateArmFwdKin::Response& res);
    bool handle_inv_kin_callback(srr_msgs::CalculateArmInvKin::Request&  req,
                                 srr_msgs::CalculateArmInvKin::Response& res);
};  // class ArmKinContainer

}   // namespace SRR

#endif  // __SRR_ARM_KIN_CONTAINER_HPP__
