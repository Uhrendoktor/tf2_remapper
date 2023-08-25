#include <tf2_remapper/Tf2Remapper.hpp>

namespace tf2_remapper
{
    TF2Remapper::TF2Remapper(const rclcpp::NodeOptions & options):
        Node("tf2_remapper", options),
        interfaces(),
        type_support(),
        type_handles()
    {
        // Declare parameters
        this->declare_parameter<std::string>("frame_mapping", "[]");
        this->declare_parameter<std::string>("topic_mapping", "["
        "    {"
        "        \"in\": \"tf_in\","
        "        \"out\": \"tf_out\""
        "    },"
        "    {"
        "        \"in\": \"tf_static_in\","
        "        \"out\": \"tf_static_out\""
        "    }"
        "]");
        this->declare_parameter<int>("topic_refresh_rate", 1000); // ms

        // parse frame_mapping as json
        this->parse_mapping(this->get_parameter("frame_mapping").as_string(), this->frame_map);

        // Parse topic_mapping as json
        this->parse_mapping(this->get_parameter("topic_mapping").as_string(), this->topic_map);

        // get topic refresh rate
        const int topic_refresh_rate = this->get_parameter("topic_refresh_rate").as_int();
        // create timer to refresh topics
        RCLCPP_INFO(this->get_logger(), "Refreshing topics every %d ms", topic_refresh_rate);
        this->timer = this->create_wall_timer(
            std::chrono::milliseconds(topic_refresh_rate),
            std::bind(&TF2Remapper::refresh_topics, this)
        );

        RCLCPP_INFO(this->get_logger(), "TF2Remapper node initialized");
    }

    TF2Remapper::~TF2Remapper()
    {
        RCLCPP_INFO(this->get_logger(), "TF2Remapper node shutting down");
        this->interfaces.clear();
        for (auto _: this->type_support){
            UNLOAD_TYPESUPPORT_LIB_CPP(_.second.second);
        }
        this->type_support.clear();
        this->type_handles.clear();
    }

    void TF2Remapper::refresh_topics() {
        // get all topics
        std::map<std::string, std::vector<std::string>> topics = this->get_topic_names_and_types();
        
        //owning variable for the new topic-names
        std::string new_topic_name;
        
        // create new subscriptions and topics
        for (auto topic = topics.cbegin(); topic != topics.cend();)
        {
            const std::string& topic_name = topic->first;

            // skip the topic if it is already registered
            if (this->interfaces.contains(topic_name)){
                ++topic;
                continue;
            }

            // skip the topic if no topic-regex match the topic-name
            if (!this->replace_mapping(topic_name, new_topic_name, topic_map, true)) {
                ++topic;
                continue;
            }

            const std::string& message_type_str = topic->second[0];
            const dynlib::message_type_t message_type = dynlib::from_message_type_string(message_type_str);
            const std::string& package_name = message_type.package_name;
            const std::string& message_name = message_type.message_name;
            // get type-support for message-type
            // check if type-support is already loaded for type
            if (!this->type_support.contains(package_name)){
                // load correct type-support lib
                void* lib = LOAD_TYPESUPPORT_LIB_CPP(package_name);
                RCLCPP_INFO(
                    this->get_logger(),
                    "Loading type-support for package: [%s]",
                    package_name.c_str()
                );
                this->type_support[package_name] = std::make_pair(
                    0,
                    std::move(lib)
                );
                // init message-type-support array
                this->type_handles[lib] = std::map<const std::string, const rosidl_message_type_support_t*>();
            }
            auto type_support = this->type_support[package_name];
            size_t& ref_count = type_support.first;
            void* type_support_lib = type_support.second;

            auto lib_type_handles = this->type_handles[type_support_lib];
            if (!lib_type_handles.contains(message_name)){
                lib_type_handles[message_name] = dynlib::cpp::get_message_type_support_handle(
                    type_support_lib,
                    message_type
                );
            }

            const rosidl_message_type_support_t* type_handle = lib_type_handles[message_name];

            std::shared_ptr<std::shared_ptr<rclcpp::GenericPublisher>> publisher = std::make_shared<std::shared_ptr<rclcpp::GenericPublisher>>();
            
            RCLCPP_INFO(
                this->get_logger(),
                "Remapping topic: [%s] -> [%s] | type: [%s]",
                topic_name.c_str(),
                new_topic_name.c_str(),
                message_type_str.c_str()
            );                
            
            // create subscription
            std::weak_ptr<std::shared_ptr<rclcpp::GenericPublisher>> w_publisher = publisher;
            rclcpp::GenericSubscription::SharedPtr subscription = this->create_generic_subscription(
                topic_name,
                message_type_str,
                this->default_qos,
                std::bind(
                    &TF2Remapper::generic_callback,
                    this,
                    std::placeholders::_1,
                    topic_name,
                    message_type_str,
                    w_publisher,
                    type_handle
                )
            );

            // get actual qos of subscription
            rclcpp::QoS qos = subscription->get_actual_qos();

            // create publisher and replace dummy publisher
            *publisher = this->create_generic_publisher(
                new_topic_name,
                message_type_str,
                qos
            );
            this->interfaces[topic_name] = std::make_pair(std::move(subscription), std::move(publisher));

            // subscription and publisher were created succesfully
            ref_count++;
        }

        // remove closed subscriptions and topics
        for (auto interface = this->interfaces.cbegin(); interface != this->interfaces.cend();)
        {
            const std::string& topic_name = interface->first;
            if (!topics.contains(topic_name)){
                
                // unload type-support libs if neccessary
                auto type_handle = this->type_support[topic_name];
                if (--type_handle.first == 0){
                    // unload lib
                    UNLOAD_TYPESUPPORT_LIB_CPP(type_handle.second);
                    this->type_support.erase(topic_name);
                    this->type_handles.erase(type_handle.second);

                    RCLCPP_INFO(
                        this->get_logger(),
                        "Unloading type-support for package: [%s]",
                        topic_name.c_str()
                    );
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "Remapping ended for topic: [%s]",
                    topic_name.c_str()
                );

                // remove subscription and publisher
                this->interfaces.erase(interface++);

                continue;
            }
            ++interface;
        }
    }

    void TF2Remapper::parse_mapping(
        const std::string &mapping,
        std::vector<std::tuple<std::regex, std::string>> &map
    ){
        nlohmann::json mapping_json = nlohmann::json::parse(mapping);
        for (auto &mapping : mapping_json)
        {
            // check if mapping contains keys "regex" or "old"
            if (mapping.contains("reg") && mapping.contains("rep")) {
                const std::string regex_str = mapping["reg"];
                const std::string replacement_str = mapping["rep"];
                const std::regex regex(regex_str);
                map.push_back(std::make_tuple(regex, replacement_str));
            } else if(mapping.contains("in") && mapping.contains("out")){
                const std::string in_str = mapping["in"];
                const std::string out_str = mapping["out"];
                // match whole word
                const std::string regex_str = "^(\\/|)" + in_str + "$";
                const std::regex regex(regex_str);
                map.push_back(std::make_tuple(regex, out_str));
            } else {
                RCLCPP_ERROR(this->get_logger(), "Mapping must contain either [\"reg\" and \"rep\"] or [\"in\" and \"out\"] key for each mapping");
            }
        }
    }

    bool TF2Remapper::replace_mapping(
        const std::string &in,
        std::string &out,
        const std::vector<std::tuple<std::regex, std::string>> &mapping,
        bool enforce_slash = false
    ){
        for (auto &kvpair : mapping)
        {
            const std::regex &regex = std::get<0>(kvpair);
            const std::string &replacement = std::get<1>(kvpair);
            if (std::regex_match(in, regex))
            {
                out = std::regex_replace(in, regex, replacement);
                if (enforce_slash && out[0] != '/'){
                    out = "/" + out;
                }
                return true;
            }
        }
        return false;
    }

    void TF2Remapper::generic_callback(
        std::shared_ptr<rclcpp::SerializedMessage> serialized_message,
        std::string topic_name_str,
        std::string message_type_str,
        std::weak_ptr<std::shared_ptr<rclcpp::GenericPublisher>> w_publisher,
        const rosidl_message_type_support_t* type_handle
    ){
        if (std::shared_ptr<std::shared_ptr<rclcpp::GenericPublisher>> publisher_ptr = w_publisher.lock()){
            if(publisher_ptr == nullptr){
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Callback for noninitialized publisher: [%s] | type: [%s]",
                    topic_name_str.c_str(),
                    message_type_str.c_str()
                );
                return;
            }
            std::shared_ptr<rclcpp::GenericPublisher> publisher = *publisher_ptr;
            if (publisher == nullptr){
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Callback for noninitialized publisher: [%s] | type: [%s]",
                    topic_name_str.c_str(),
                    message_type_str.c_str()
                );
                return;
            }

            // deserialize message
            dynlib::cpp::MessageData message_data(type_handle);
            dynlib::cpp::Message message(
                type_handle,
                message_data.get_data(),
                serialized_message.get()
            );

            // get Header
            std_msgs::msg::Header* header = message.get_header();
            if (header != nullptr){
                this->replace(*header);
            }

            if (message_type_str == "tf2_msgs/msg/TFMessage"){
                tf2_msgs::msg::TFMessage* tf_message = message.cast<tf2_msgs::msg::TFMessage>();
                this->replace(*tf_message);
            }

            // publish message
            publisher->publish(message.serialize());

        } else {
            RCLCPP_ERROR(
                this->get_logger(),
                "Callback publisher went out of scope: [%s] | type: [%s]",
                topic_name_str.c_str(),
                message_type_str.c_str()
            );
        }
    }

    void TF2Remapper::replace(tf2_msgs::msg::TFMessage &msg)
    {
        for (auto &transform : msg.transforms)
        {
            this->replace_mapping(
                transform.child_frame_id,
                transform.child_frame_id,
                this->frame_map,
                false
            );

            this->replace_mapping(
                transform.header.frame_id,
                transform.header.frame_id,
                this->frame_map,
                false
            );
        }
    }

    void TF2Remapper::replace(std_msgs::msg::Header &header)
    {
        this->replace_mapping(
            header.frame_id,
            header.frame_id,
            this->frame_map,
            false
        );
    }
} // namespace tf2_remapper

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<tf2_remapper::TF2Remapper>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}