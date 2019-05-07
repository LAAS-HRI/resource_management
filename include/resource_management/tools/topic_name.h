#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_TOOLS_TOPIC_NAME_H_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_TOOLS_TOPIC_NAME_H_

#include <string>
#include <regex>

namespace resource_management
{
namespace tools
{
template<class MessageType>
std::string topic_name(const std::string &reactive_buffer_name, const std::string &ns=""){
    std::string name=ns;
    if (!name.empty() && name[name.size()-1]!='/') name += '/';
    name += reactive_buffer_name + "/";
    std::regex regex_name("^N\\d+(.*)\\d+(.*)_ISaIvEEE$");
    std::smatch match;
    std::string type_id = typeid(MessageType).name();
    if(std::regex_match(type_id, match, regex_name))
        name += match[1].str() + "_" + match[2].str();
    else
        name += typeid(MessageType).name();

    return name;
}

}
}

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_TOOLS_TOPIC_NAME_H_
