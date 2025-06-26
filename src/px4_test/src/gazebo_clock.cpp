#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/float64.hpp>

class GazeboClockProcessor : public rclcpp::Node
{
public:
    GazeboClockProcessor()
    : Node("gazebo_clock_processor")
    {
        // Create subscription to /clock topic
        clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 10,
            std::bind(&GazeboClockProcessor::clock_callback, this, std::placeholders::_1));
        
        // Create publisher for processed time
        time_pub_ = this->create_publisher<std_msgs::msg::Float64>("/simulation_time", 10);
        
        latest_time_ = 0.0;
    }

private:
    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        // Convert to seconds
        latest_time_ = static_cast<double>(msg->clock.sec) + 
                      static_cast<double>(msg->clock.nanosec) * 1e-9;
        
        // Create and publish message
        auto time_msg = std_msgs::msg::Float64();
        time_msg.data = latest_time_;
        time_pub_->publish(time_msg);
        
        RCLCPP_INFO(this->get_logger(), "Published simulation time: %.3f s", latest_time_);
    }

    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr time_pub_;
    double latest_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboClockProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}   