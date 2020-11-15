#ifndef __SRR_STRING_UTILS_HPP__
#define __SRR_STRING_UTILS_HPP__

#include <string>

namespace SRR {

bool split_string_at(std::string input, std::string delim, std::string& left, std::string& right)
{
    size_t idx = -1;
    if ((idx = input.find(delim)) == std::string::npos)
    {
        left  = input;
        right = "";
        return false;
    }
    else
    {
        left  = input.substr(0, idx);
        right = input.substr(idx + 1);
        return true;
    }
}

}	// namespace SRR

#endif	// __SRR_STRING_UTILS_HPP__
