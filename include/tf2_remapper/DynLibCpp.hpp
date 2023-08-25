#ifndef DYNLIBCPP_HPP
#define DYNLIBCPP_HPP

#include "DynLib.hpp"
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

namespace dynlib {
    namespace cpp {
        #define USING_CPP(library, typename) \
            using typename = library ## _cpp :: typename;

        #define ROSIDL_TYPESUPPORT_CPP "rosidl_typesupport_cpp"

        #define TYPESUPPORT_HANDLE_LIB_CPP(package_name) \
            "lib" + package_name + __ + ROSIDL_TYPESUPPORT_CPP ".so"

        #define LOAD_TYPESUPPORT_LIB_CPP(package_name) \
            dlopen(std::string(TYPESUPPORT_HANDLE_LIB_CPP(package_name)).c_str(), RTLD_LAZY)

        #define UNLOAD_TYPESUPPORT_LIB_CPP(lib) \
            dlclose(lib)
    
        #define MANGLE_SUBNAMESPACE(token) \
            std::to_string((token).size()) + (token)
          
        #define GET_MESSAGE_TYPE_SUPPORT_HANDLE_FUNC_CPP(package_name, subfolder, message_name) \
            ROSIDL_TYPESUPPORT_CPP __ GET_MESSAGE_TYPE_SUPPORT_HANDLE __ + package_name + __ + subfolder + __ + message_name

        USING_CPP(rosidl_typesupport_introspection, MessageMembers)
        USING_CPP(rosidl_typesupport_introspection, MessageMember)

        const rosidl_message_type_support_t* get_message_type_support_handle(
            void* typesupport_lib,
            const message_type_t &message_type
        ){
            if (!typesupport_lib) {
                return nullptr;
            }
            
            std::string get_message_type_support_handle_func_name = GET_MESSAGE_TYPE_SUPPORT_HANDLE_FUNC_CPP(
                message_type.package_name,
                message_type.subfolder,
                message_type.message_name
            );

            get_message_type_support_handle_func_t get_message_type_support_handle_func = reinterpret_cast<get_message_type_support_handle_func_t>(dlsym(
                typesupport_lib,
                get_message_type_support_handle_func_name.c_str()
            ));

            if (get_message_type_support_handle_func == nullptr) {
                return nullptr;
            }

            return get_message_type_support_handle_func();
        }

        const MessageMembers* get_message_members(
            const rosidl_message_type_support_t* type_support
        ){
            // rosidl's get_message_typesupport_handle
            const rosidl_message_type_support_t* introspection_type_support = get_message_typesupport_handle(
                type_support,
                rosidl_typesupport_introspection_cpp::typesupport_identifier
            );
            return reinterpret_cast<const MessageMembers*>(introspection_type_support->data);
        }

        class MessageData{
            public:
                MessageData(
                    const rosidl_message_type_support_t* type_support
                ):
                type_support(type_support),
                message_members(get_message_members(type_support)),
                allocator(rcutils_get_default_allocator()) {
                    this->data = reinterpret_cast<uint8_t*>(
                        this->allocator.allocate(
                            message_members->size_of_,
                            this->allocator.state
                        )
                    );
                    this->message_members->init_function(this->data, rosidl_runtime_cpp::MessageInitialization::ALL);
                }

                ~MessageData(){
                    this->message_members->fini_function(this->data);
                    this->allocator.deallocate(this->data, this->allocator.state);
                }

                uint8_t* get_data(){
                    return this->data;
                }

            private:
                const rosidl_message_type_support_t* type_support;
                const MessageMembers* message_members;
                rcutils_allocator_t allocator;
                uint8_t* data;
        };

        class Message {
            public:
                Message(
                    const rosidl_message_type_support_t* type_support,
                    uint8_t* data
                ): 
                type_support(type_support),
                members(get_message_members(type_support)),
                data(data)
                {
                }

                Message(
                    const rosidl_message_type_support_t* type_support,
                    uint8_t* data,
                    const rclcpp::SerializedMessage* serialized_message
                ):
                // allocates memory for the message 
                Message(
                    type_support,
                    data
                ){
                    //check for serialized message capacity and size
                    if(serialized_message->capacity() == 0u || serialized_message->size() == 0u){
                        throw std::runtime_error("Serialized message is empty");
                    }
                    if(RMW_RET_OK != rmw_deserialize(
                        &serialized_message->get_rcl_serialized_message(),
                        type_support,
                        data
                    )){
                        throw std::runtime_error("Failed to deserialize message");
                    }
                }

                rclcpp::SerializedMessage serialize(){
                    rclcpp::SerializedMessage serialized_message;
                    if(RMW_RET_OK != rmw_serialize(
                        this->data,
                        this->type_support,
                        &serialized_message.get_rcl_serialized_message()
                    )){
                        throw std::runtime_error("Failed to serialize message");
                    }
                    return serialized_message;
                }

                size_t get_field(
                    const char* field_name
                ){
                    for (size_t i = 0; i < this->members->member_count_; ++i) {
                        const MessageMember &member = this->members->members_[i];
                        if (!strcmp(member.name_, field_name)){
                            return i;
                        }
                    }
                    return (size_t)-1;
                }

                Message from_field(
                    size_t field_index
                ){
                    const MessageMember &member = this->members->members_[field_index];
                    return Message(
                        member.members_,
                        &this->data[member.offset_]
                    );
                }

                uint8_t* get_data(){
                    return this->data;
                }

                template <typename T> T* cast(){
                    return reinterpret_cast<T*>(this->data);
                } 

                std_msgs::msg::Header* get_header(){
                    const char* header_name = "header";
                    size_t field_index = this->get_field(header_name);
                    if(field_index == (size_t)-1){
                        return nullptr;
                    }
                    return this->from_field(field_index).cast<std_msgs::msg::Header>();
                }

            private:
                const rosidl_message_type_support_t* type_support;
                const MessageMembers* members;
                uint8_t* data;
        };
    }
}

#endif // DYNLIBCPP_HPP