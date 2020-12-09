#include "srr_kinematics/ArmKinContainer.hpp"

#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>

#include "srr_msgs/ArmJointPose.h"

#include "srr_utils/math_utils.hpp"

double SRR::ArmKinContainer::s(double rad)
{
    return std::sin(rad);
}

double SRR::ArmKinContainer::c(double rad)
{
    return std::cos(rad);
}

double SRR::ArmKinContainer::at2(double y, double x)
{
    return std::atan2(y, x);
}

SRR::ArmKinContainer::ArmKinContainer()
{
}

SRR::ArmKinContainer& SRR::ArmKinContainer::add_param(std::string param, double value)
{
    dh.emplace(param, value);
    return *this;
}

bool SRR::ArmKinContainer::handle_fwd_kin_callback(srr_msgs::CalculateArmFwdKin::Request&  req,
                                                   srr_msgs::CalculateArmFwdKin::Response& res)
{
    double d1 = dh["d1"], d2 = dh["d2"], a2 = dh["a2"], d3 = dh["d3"], d45 = dh["d4"] + dh["d5"];
    double q1 = req.joint_pose.q[0], q2 = req.joint_pose.q[1], q3 = req.joint_pose.q[2], q4 = req.joint_pose.q[3];

    res.solution.gamma = q2 + q3 + q4;

    double xz0_projection = (d2 * s(q2)) + (a2 * c(q2)) + (d3 * s(q2 + q3)) + (d45 * s(res.solution.gamma));
    res.solution.x = c(q1) * xz0_projection;
    res.solution.y = s(q1) * xz0_projection;
    res.solution.z = d1 + (d2 * c(q2)) - (a2 * s(q2)) + (d3 * c(q2 + q3)) + (d45 * c(res.solution.gamma));

    return true;
}

bool SRR::ArmKinContainer::handle_inv_kin_callback(srr_msgs::CalculateArmInvKin::Request&  req,
                                                   srr_msgs::CalculateArmInvKin::Response& res)
{
    double d1 = dh["d1"], d2 = dh["d2"], a2 = dh["a2"], d3 = dh["d3"], d45 = dh["d4"] + dh["d5"], pi = SRR::pi();
    double x_0n = req.cart_pose.x, y_0n = req.cart_pose.y, z_0n = req.cart_pose.z, gamma = req.cart_pose.gamma;

    srr_msgs::ArmJointPose pose;

    // Calculate theta 1
    // The range for this joint is [-pi/2. pi/2], which is the range of atan2 as well, so no check is necessary
    double q1 = at2(y_0n, x_0n);

    // Calculate P_in from P_0n and T_0i
    tf::Transform T_01(
        tf::Matrix3x3(c(q1), -s(q1), 0,
                      s(q1),  c(q1), 0,
                      0,      0,     1),
        tf::Vector3(0, 0, d1)
    );
    static tf::Transform T_12(
        tf::Matrix3x3(1,  0, 0,
                      0,  0, 1,
                      0, -1, 0),
        tf::Vector3(0, 0, 0)
    );
    static tf::Transform T_2i(
        tf::Matrix3x3( 0, 1, 0,
                      -1, 0, 0,
                       0, 0, 1),
        tf::Vector3(0, 0, 0)
    );
    tf::Vector3 P_0n(x_0n, y_0n, z_0n);
    tf::Vector3 P_in = (T_01 * T_12 * T_2i).inverse() * P_0n;
    double x_in = P_in.getX(), y_in = P_in.getY();

    // Calculate x-prime and y-prime
    double xp = x_in - (d45 * c(gamma));
    double yp = y_in - (d45 * s(gamma));

    // Calculate P, Q, and R
    double P = -2 * ((xp * d2) + (yp * a2));
    double Q = 2 * ((xp * a2) - (yp * d2));
    double R = (xp * xp) * (yp * yp) + (d2 * d2) + (a2 * a2) + (d3 * d3);

    // Calculate delta and both possible values of theta 2
    double mag_PQ = std::sqrt((P * P) + (Q * Q));
    double delta = at2(Q / mag_PQ, P / mag_PQ);
    double q2_inf = std::acos(-R / mag_PQ);
    double q2_1 = delta + q2_inf, q2_2 = delta - q2_inf;

    // Check values for theta 2
    double min = 0, max = 2 * pi / 3;
    bool q2_1_valid = ((q2_1 > min) && (q2_1 < max)), q2_2_valid = ((q2_2 > min) && (q2_2 < max));

    // Calculate both possible values of theta 3
    double q3_1 = at2(yp - (d2 * s(q2_1)) - (a2 * c(q2_1)), xp - (d2 * c(q2_1)) + (a2 * s(q2_1))) - q2_1;
    double q3_2 = at2(yp - (d2 * s(q2_2)) - (a2 * c(q2_2)), xp - (d2 * c(q2_2)) + (a2 * s(q2_2))) - q2_2;

    // Check values for theta 3
    min = 0; max = pi;
    bool q3_1_valid = ((q3_1 > min) && (q3_1 < max)), q3_2_valid = ((q3_2 > min) && (q3_2 < max));

    // Calculate both possible values of theta 4
    double q4_1 = gamma - q2_1 - q3_1;
    double q4_2 = gamma - q2_2 - q3_2;

    // Check values for theta 4
    max = 2 * pi / 3;
    min = -max;
    bool q4_1_valid = ((q4_1 > min) && (q4_1 < max)), q4_2_valid = ((q4_2 > min) && (q4_2 < max));

    // Add valid solutions
    if (q2_1_valid && q3_1_valid && q4_1_valid)
    {
        srr_msgs::ArmJointPose pose;
        pose.q[0] = q1;
        pose.q[1] = q2_1;
        pose.q[2] = q3_1;
        pose.q[3] = q4_1;
        res.solution.push_back(pose);
    }
    if (q2_2_valid && q3_2_valid && q4_2_valid)
    {
        srr_msgs::ArmJointPose pose;
        pose.q[0] = q1;
        pose.q[1] = q2_2;
        pose.q[2] = q3_2;
        pose.q[3] = q4_2;
        res.solution.push_back(pose);
    }

    return true;
}
