#ifndef __SRR_JOINT_OFFSETS_COLLECTION_HPP__
#define __SRR_JOINT_OFFSETS_COLLECTION_HPP__

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <assert.h>

#include "srr_msgs/CalculatePositionWithOffsets.h"

namespace SRR {

class JointOffsetsCollection {
public:
    enum JointDirectionEnum {NEGATIVE = -1, POSITIVE = +1};

private:
    std::unordered_map<std::string, double>             joint_name_offset_map;
    std::unordered_map<std::string, JointDirectionEnum> joint_name_direction_map;

    static bool split_string_at(std::string input, std::string delim, std::string& left, std::string& right);

public:
    JointOffsetsCollection(std::string list_joint_name_and_offset, std::string list_joint_name_and_direction);

    bool handle_callback(srr_msgs::CalculatePositionWithOffsets::Request&  req,
                         srr_msgs::CalculatePositionWithOffsets::Response& res);
};	// class JointOffsetsCollection

}	// namespace SRR

#endif	// __SRR_JOINT_OFFSETS_COLLECTION_HPP__
