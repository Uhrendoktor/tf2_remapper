#ifndef TF2_REMAPPER__TF2_REMAPPER_HPP_
#define TF2_REMAPPER__TF2_REMAPPER_HPP_

#include <string>
#include <vector>
#include <tuple>
#include <regex>
#include <iostream>
#include <iterator>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"

#include "tf2_msgs/msg/tf_message.hpp"
#include "DynLibC.hpp"
#include "DynLibCpp.hpp"

namespace tf2_remapper
{
    class TF2Remapper : public rclcpp::Node
    {
    public:
        explicit TF2Remapper(const rclcpp::NodeOptions & options);
        ~TF2Remapper();

    private:
        void parse_mapping(
            const std::string &mapping,
            std::vector<std::tuple<std::regex, std::string>> &map
        );
        bool replace_mapping(
            const std::string &in,
            std::string &out,
            const std::vector<std::tuple<std::regex, std::string>> &mapping,
            bool enforce_slash
        );
        void generic_callback(
            std::shared_ptr<rclcpp::SerializedMessage> serialized_message,
            std::string topic_name_str,
            std::string message_type_str,
            std::weak_ptr<std::shared_ptr<rclcpp::GenericPublisher>> w_publisher,
            const rosidl_message_type_support_t* type_handle
        );
        void replace(tf2_msgs::msg::TFMessage &msg);
        void replace(std_msgs::msg::Header &header);

        void refresh_topics();

        // subscriptions and publishers of generic type
        std::map<const std::string, std::pair<rclcpp::GenericSubscription::SharedPtr, std::shared_ptr<rclcpp::GenericPublisher::SharedPtr>>> interfaces;

        // map of all hot-loaded type support handles
        std::map<const std::string, std::pair<size_t, void*>> type_support;
        std::map<void*, std::map<const std::string, const rosidl_message_type_support_t*>> type_handles;

        std::vector<std::tuple<std::regex, std::string>> frame_map;
        std::vector<std::tuple<std::regex, std::string>> topic_map; 

        const rclcpp::QoS default_qos = rclcpp::QoS(1);

        rclcpp::TimerBase::SharedPtr timer;
    };
} // namespace tf2_remapper

#endif // TF2_REMAPPER__TF2_REMAPPER_HPP_

