#include "connected_explorers_message_broker/BrokerClient.hpp"


BrokerClient::BrokerClient(
    std::shared_ptr<rclcpp::Node> node,
    int number_of_robots,
    std::string inbox_topic_name,
    int qos_profile
):node_(node),number_of_robots_(number_of_robots),inbox_topic_name_(inbox_topic_name),qos_profile_(qos_profile){
    InitInboxSubscriber();
}

BrokerClient::~BrokerClient()
{
}

void BrokerClient::InitInboxSubscriber(){
    inbox_subscriber_list_=node_->
        create_subscription<connected_explorers_interfaces::msg::SyncState>(
            inbox_topic_name_,
            qos_profile_,
            std::bind(&BrokerClient::InboxSubscriberCallback,this,std::placeholders::_1)
        );
}

void BrokerClient::InboxSubscriberCallback(const connected_explorers_interfaces::msg::SyncState::SharedPtr msg){
    int list_index = msg->robot_index;
    UpdateLastMessageRegister(list_index,msg);

}

void BrokerClient::UpdateLastMessageRegister(int index, connected_explorers_interfaces::msg::SyncState::SharedPtr msg){
    std::lock_guard<std::mutex> lock(message_mutex_);
    last_messages_[index] = msg;
}

std::vector<connected_explorers_interfaces::msg::SyncState::SharedPtr> BrokerClient::GetLastMessages(){
    std::lock_guard<std::mutex> lock(message_mutex_);
    return last_messages_;
}

