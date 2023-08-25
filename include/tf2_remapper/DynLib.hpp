#ifndef DYNLIB_HPP
#define DYNLIB_HPP

#include <dlfcn.h>

#define GET_MESSAGE_TYPE_SUPPORT_HANDLE "get_message_type_support_handle"
#define __ "__"

extern "C" {
    typedef const rosidl_message_type_support_t * (* get_message_type_support_handle_func_t)();
}

namespace dynlib {
    struct message_type_t {
        const std::string package_name;
        const std::string message_name;
        const std::string subfolder;
    };

    const message_type_t from_message_type_string(const std::string s)
    {
        // TODO: maybe allow for other subfolder structures. Check if rosidl allows for that
        std::regex regex("([^\\/]{1,})\\/msg\\/([^\\/]{1,})");
        std::smatch match;
        if (std::regex_search(s, match, regex))
        {
            return message_type_t{
                match[1].str(),
                match[2].str(),
                "msg"
            };
        } 
        else
        {
            throw std::runtime_error("Could not parse message type string");
        }
    }
}

#endif // DYNLIB_HPP