#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <memory>

#include <rosgraph_msgs/msg/clock.hpp>

#include "px4_test/frame_transforms.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <array> 
#include <std_msgs/msg/float64.hpp>


class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        // Import parameters from the YAML file
        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        RCLCPP_INFO(this->get_logger(), "use_sim_time: %s", use_sim_time ? "true" : "false");
         
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        vehicle_odometry_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            qos,
            std::bind(&OffboardControl::odom_callback, this, std::placeholders::_1)
        );

        // Create subscription to /clock topic
        clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 10,
            std::bind(&OffboardControl::clock_callback, this, std::placeholders::_1));
        
        // Create publisher for processed time
        time_pub_ = this->create_publisher<std_msgs::msg::Float64>("/simulation_time", 10);

        offboard_setpoint_counter_ = 0;

        auto timer_callback = [this]() -> void {

            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                // Arm the vehicle
                this->arm();
            }

            if (offboard_setpoint_counter_ == 100) {
                // Command the vehicle to land
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0, 0);
                RCLCPP_INFO(this->get_logger(), "Landing command sent");
            }

            if (offboard_setpoint_counter_ == 300) {
                // Disarm the vehicle after 20 setpoints
                this->disarm();
            }

            // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode();
            publish_trajectory_setpoint();

            // stop the counter after reaching 151
            if (offboard_setpoint_counter_ < 301) {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
    }

    void arm();
    void disarm();

private:
    void odom_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscription_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        // Convert to seconds
        latest_time_ = static_cast<double>(msg->clock.sec) + 
                      static_cast<double>(msg->clock.nanosec) * 1e-9;
        
        // Create and publish message
        auto time_msg = std_msgs::msg::Float64();
        time_msg.data = latest_time_;
        time_pub_->publish(time_msg);
        
        // RCLCPP_INFO(this->get_logger(), "Published simulation time: %.3f s", latest_time_);
    }

    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr time_pub_;
    double latest_time_;
};
 

void OffboardControl::arm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}


void OffboardControl::disarm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -5.0};
    msg.yaw = -3.14; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}


void OffboardControl::odom_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
{
    timestamp_ = msg->timestamp;
    Eigen::Vector3d position(msg->position[0], msg->position[1], msg->position[2]);
    Eigen::Quaterniond q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    Eigen::Vector3d euler_angles = px4_ros_com::frame_transforms::utils::quaternion::quaternion_to_euler(q);

    // RCLCPP_INFO(
    //     this->get_logger(), 
    //     "Received Odometry: Position [x: %f, y: %f, z: %f], Angles [roll: %f, pitch: %f, yaw: %f]",
    //     position[0], position[1], position[2],
    //     euler_angles[0], euler_angles[1], euler_angles[2]
    // );
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}
