#ifndef BROKER_CLIENT_HPP
#define BROKER_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"

//custom messages ---
#include "connected_explorers_interfaces/msg/sync_state.hpp"

#define DEFAULT_INBOX_TOPIC_NAME "inbox"

class BrokerClient
{
private:
    std::shared_ptr<rclcpp::Node> node_;
    int number_of_robots_;
    std::string inbox_topic_name_;
    int qos_profile_;


    std::mutex message_mutex_; 

    rclcpp::Subscription<connected_explorers_interfaces::msg::SyncState>::SharedPtr inbox_subscriber_list_;

    std::vector<connected_explorers_interfaces::msg::SyncState::SharedPtr> last_messages_;

public:
    BrokerClient(
        std::shared_ptr<rclcpp::Node> node,
        int number_of_robots,
        std::string inbox_topic_name,
        int qos_profile
    );
    ~BrokerClient();


private:
    void InitInboxSubscriber();
    void InboxSubscriberCallback(const connected_explorers_interfaces::msg::SyncState::SharedPtr msg);
    void UpdateLastMessageRegister(int index, connected_explorers_interfaces::msg::SyncState::SharedPtr msg);
    std::vector<connected_explorers_interfaces::msg::SyncState::SharedPtr> GetLastMessages();

};








#endif //BROKER_CLIENT_HPP