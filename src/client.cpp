/*
 * GStreamer toggle service client
 * Objectives:
 *  - Get inputs from user
 *  - Send request to service server based on input
 *
 * Default state: PAUSED, 600x400
 * Inputs: "p" (play), "s" (stop), "r" (resolution)
 * Control logic will be done on server (in case we have more clients)
 * Invalid inputs will be ignored! (Print help message)
 *
 * Note:
 * Node can't be run with launchfile due to buffering of stdout
 * https://github.com/ros2/rclcpp/issues/982#issuecomment-583044839
 */

#include <iostream>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
// #include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
using SetBool = std_srvs::srv::SetBool;
// using Empty = std_srvs::srv::Empty;
using std::placeholders::_1;

class ClientNode : public rclcpp::Node
{
public:
    ClientNode() : Node("client_node") {}

    bool get_user_input()
    {
        std::string input;
        std::cout << "Enter info: ";
        std::getline(std::cin, input);
        RCLCPP_INFO_STREAM(this->get_logger(), "Input: " << input);
        return this->handle_user_input(input);
    }

    bool wait_for_services()
    {
        while (!client_resolution_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        while (!client_state_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        return true;
    }

    bool get_service_done() { return this->service_done; }

private:
    rclcpp::Client<SetBool>::SharedPtr client_resolution_ = this->create_client<SetBool>("change_resolution");
    rclcpp::Client<SetBool>::SharedPtr client_state_ = this->create_client<SetBool>("change_state");

    bool service_done = false;

    bool handle_user_input(std::string in)
    {
        this->service_done = false;
        if ("p" == in)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Play!");
            auto request = std::make_shared<SetBool::Request>();
            request->data = true;
            auto result_ = client_state_->async_send_request(request, std::bind(&ClientNode::state_callback, this, _1));
            // RCLCPP_INFO_STREAM(this->get_logger(), result_.get()->success);
            return true; // todo
        }
        else if ("s" == in)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Stop!");
            auto request = std::make_shared<SetBool::Request>();
            request->data = false;
            auto result_ = client_state_->async_send_request(request, std::bind(&ClientNode::state_callback, this, _1));
            // RCLCPP_INFO_STREAM(this->get_logger(), result_.get()->success);
            return true;
        }
        else if ("R" == in)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Resolution: 600x400!");
            auto request = std::make_shared<SetBool::Request>();
            request->data = true;
            auto result_ = client_resolution_->async_send_request(request, std::bind(&ClientNode::resolution_callback, this, _1));
            return true;
        }
        else if ("r" == in)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Resolution: 400x400!");
            auto request = std::make_shared<SetBool::Request>();
            request->data = false;
            auto result_ = client_resolution_->async_send_request(request, std::bind(&ClientNode::resolution_callback, this, _1));
            return true;
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Invalid input!");
            return false;
        }
    }

    void resolution_callback(rclcpp::Client<SetBool>::SharedFuture future)
    {
        auto status = future.wait_for(1s);
        if (status == std::future_status::ready)
        {
            service_done = true;
            if (!future.get()->success)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "-- Message: " << future.get()->message.c_str());
            }
            return;
        }
    }

    void state_callback(rclcpp::Client<SetBool>::SharedFuture future)
    {
        auto status = future.wait_for(1s);
        if (status == std::future_status::ready)
        {
            service_done = true;
            if (!future.get()->success)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "-- Message: " << future.get()->message.c_str());
            }
            return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<ClientNode> node = std::make_shared<ClientNode>();

    bool input_valid = true;

    if (not(node->wait_for_services()))
    {
        return 1;
    }

    RCLCPP_INFO_STREAM(node->get_logger(), "Client node up!");

    // while (rclcpp::ok())
    while (input_valid)
    {
        input_valid = node->get_user_input();
        if (input_valid)
        {
            while (!node->get_service_done())
            {
                rclcpp::spin_some(node);
            }
        }
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "Shutting down!");
    rclcpp::shutdown();
    return 0;
}