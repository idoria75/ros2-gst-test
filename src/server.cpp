/*
 * GStreamer toggle service server
 * Objectives:
 *  - Start streaming videotestsrc upon request (600x400 @ 30FPS, UDP)
 *  - Change aspect to 400x400 upon another request
 *
 * Default state: PAUSED, 600x400
 */

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

// using namespace std::chrono_literals;
using SetBool = std_srvs::srv::SetBool;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node
{
public:
    ServerNode() : Node("server_node")
    {
        // Bind attaches the service call to the class method state_callback
        // Placeholders
        state_srv_ = create_service<SetBool>("change_state", std::bind(&ServerNode::state_callback, this, _1, _2));
        resolution_srv_ = create_service<SetBool>("change_resolution", std::bind(&ServerNode::resolution_callback, this, _1, _2));
    }

private:
    bool resolution_ = true; // True if 600x400, False if 400x400
    bool state_ = 0;         // True if playing, False if paused

    rclcpp::Service<SetBool>::SharedPtr state_srv_;
    rclcpp::Service<SetBool>::SharedPtr resolution_srv_;

    void state_callback(
        const std::shared_ptr<SetBool::Request> request,
        const std::shared_ptr<SetBool::Response> response)
    {
        if (request->data != state_)
        {
            state_ = request->data;
            response->success = true;
            RCLCPP_INFO_STREAM(this->get_logger(), "State changed to: " << request->data);
        }
        else
        {
            std::string msg = "State did not change";
            RCLCPP_INFO_STREAM(this->get_logger(), msg);
            response->success = false;
            response->message = msg;
        }
    }

    void resolution_callback(
        const std::shared_ptr<SetBool::Request> request,
        const std::shared_ptr<SetBool::Response> response)
    {
        if (request->data != resolution_)
        {
            resolution_ = request->data;
            response->success = true;
            RCLCPP_INFO_STREAM(this->get_logger(), "Resolution changed to: " << request->data);
        }
        else
        {
            std::string msg = "Resolution did not change";
            RCLCPP_INFO_STREAM(this->get_logger(), msg);
            response->success = false;
            response->message = msg;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = std::make_shared<ServerNode>();

    RCLCPP_INFO_STREAM(node->get_logger(), "Server node up!");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}