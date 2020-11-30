#ifndef __SRR_ROS_UTILS_HPP__
#define __SRR_ROS_UTILS_HPP__

#include <tf/LinearMath/Vector3.h>
#include "geometry_msgs/Vector3.h"

#include <tf/LinearMath/Matrix3x3.h>
#include "srr_msgs/RotationMatrix.h"

namespace SRR {

void popROSMsg(tf::Vector3 in, geometry_msgs::Vector3& out)
{
    out.x = in.x();
    out.y = in.y();
    out.z = in.z();
}

void popROSMsg(tf::Matrix3x3 in, srr_msgs::RotationMatrix& out)
{
    out.r1[0] = in[0].x();
    out.r1[1] = in[0].y();
    out.r1[2] = in[0].z();
    out.r2[0] = in[1].x();
    out.r2[1] = in[1].y();
    out.r2[2] = in[1].z();
    out.r3[0] = in[2].x();
    out.r3[1] = in[2].y();
    out.r3[2] = in[2].z();
}

}	// namespace SRR

#endif	// __SRR_ROS_UTILS_HPP__
