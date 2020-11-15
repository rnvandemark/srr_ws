#include "srr_kinematics/JointOffsetsCollection.hpp"

#include "srr_utils/math_utils.hpp"
#include "srr_utils/string_utils.hpp"

#include <assert.h>

SRR::JointOffsetsCollection::JointOffsetsCollection(std::string list_joint_name_and_offset, std::string list_joint_name_and_direction)
{
    std::string input = list_joint_name_and_offset, next_pair, remaining_pairs, joint_name, value;
    bool continue_loop = true;
    do
    {
        continue_loop = SRR::split_string_at(input, ",", next_pair, remaining_pairs);
        assert(SRR::split_string_at(next_pair, "=", joint_name, value));
        assert(joint_name_offset_map.find(joint_name) == joint_name_offset_map.end());
        joint_name_offset_map[joint_name] = SRR::rads(std::stod(value));
        input = remaining_pairs;
    } while (continue_loop);

    input = list_joint_name_and_direction;
    continue_loop = true;
    do
    {
        continue_loop = SRR::split_string_at(input, ",", next_pair, remaining_pairs);
        assert(SRR::split_string_at(next_pair, "=", joint_name, value));
        assert((joint_name_direction_map.find(joint_name) == joint_name_direction_map.end())
                && (joint_name_offset_map.find(joint_name) != joint_name_offset_map.end()));
        joint_name_direction_map[joint_name] = static_cast<JointDirectionEnum>(std::stoi(value));
        input = remaining_pairs;
    } while (continue_loop);

    assert(joint_name_offset_map.size() == joint_name_direction_map.size());
}

bool SRR::JointOffsetsCollection::handle_callback(srr_msgs::CalculatePositionWithOffsets::Request&  req,
                                                  srr_msgs::CalculatePositionWithOffsets::Response& res)
{
    if (req.joint_names.size() != req.goal_positions.size())
    {
        return false;
    }

    res.joint_positions.resize(req.joint_names.size());
    for (size_t i = 0; i < req.joint_names.size(); i++)
    {
        std::string joint_name = req.joint_names[i];
        if (joint_name_offset_map.find(joint_name) == joint_name_offset_map.end())
        {
            return false;
        }
        else
        {
            double goal_position = req.goal_positions[i];
            double joint_offset  = joint_name_offset_map[joint_name];
            JointDirectionEnum joint_direction = joint_name_direction_map[joint_name];
            switch (joint_direction)
            {
                case JointDirectionEnum::NEGATIVE:
                    res.joint_positions[i] = joint_offset - goal_position;
                    break;
                case JointDirectionEnum::POSITIVE:
                    res.joint_positions[i] = goal_position - joint_offset;
                    break;
                default:
                    return false;
            }
        }
    }

    return true;
}
