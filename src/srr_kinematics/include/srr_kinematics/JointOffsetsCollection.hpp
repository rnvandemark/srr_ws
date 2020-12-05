#ifndef __SRR_JOINT_OFFSETS_COLLECTION_HPP__
#define __SRR_JOINT_OFFSETS_COLLECTION_HPP__

#include "srr_msgs/CalculatePositionWithOffsets.h"
#include "srr_msgs/GetDirectionOfRotation.h"

#include <string>
#include <unordered_map>

namespace SRR {

class JointOffsetsCollection {
public:
    enum JointDirectionEnum {
        NEGATIVE = srr_msgs::GetDirectionOfRotation::Request::NEGATIVE,
        POSITIVE = srr_msgs::GetDirectionOfRotation::Request::POSITIVE
    };

private:
    std::unordered_map<std::string, double>             joint_name_offset_map;
    std::unordered_map<std::string, JointDirectionEnum> joint_name_direction_map;

public:
    JointOffsetsCollection(std::string list_joint_name_and_offset, std::string list_joint_name_and_direction);

    bool handle_calculate_position_callback(srr_msgs::CalculatePositionWithOffsets::Request&  req,
                                            srr_msgs::CalculatePositionWithOffsets::Response& res);

    bool handle_direction_of_rotation_callback(srr_msgs::GetDirectionOfRotation::Request&  req,
                                               srr_msgs::GetDirectionOfRotation::Response& res);
};	// class JointOffsetsCollection

}	// namespace SRR

#endif	// __SRR_JOINT_OFFSETS_COLLECTION_HPP__
