#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include <vector>

class LocalZeroingNode : public rclcpp::Node {
public:
    LocalZeroingNode() : Node("local_zeroing_node"), origin_set_(false) {
        // Define Best Effort QoS profile (matches MAVROS)
        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort();

        // Subscribe to MAVROS local position with Best Effort QoS
        local_position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mavros/local_position/odom", qos_profile,
            std::bind(&LocalZeroingNode::localPositionCallback, this, std::placeholders::_1));

        // Publisher for zeroed position
        zeroed_position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/local_zeroed_position", 10);

        RCLCPP_INFO(this->get_logger(), "Local Zeroing Node Initialized. Processing data for 5 seconds...");
        start_time_ = this->now();  // Record start time
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_position_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr zeroed_position_pub_;

    geometry_msgs::msg::Point origin_;
    bool origin_set_;
    std::vector<geometry_msgs::msg::Point> position_buffer_;
    rclcpp::Time start_time_;

    void localPositionCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rclcpp::Duration elapsed_time = this->now() - start_time_;

        // Collect data for 5 seconds while still publishing zeroed positions
        if (elapsed_time.seconds() < 5.0) {
            position_buffer_.push_back(msg->pose.pose.position);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "Processing data... %zu points gathered", position_buffer_.size());

            // Publish zeroed position temporarily (without finalized origin)
            geometry_msgs::msg::PoseStamped temp_msg;
            temp_msg.header.stamp = this->get_clock()->now();
            temp_msg.header.frame_id = "map";
            temp_msg.pose.position = msg->pose.pose.position;  // Pass raw position
            temp_msg.pose.orientation = msg->pose.pose.orientation;
            zeroed_position_pub_->publish(temp_msg);

            return;
        }

        // Set origin once after 5 seconds
        if (!origin_set_) {
            // Compute the average position
            double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
            for (const auto& pos : position_buffer_) {
                sum_x += pos.x;
                sum_y += pos.y;
                sum_z += pos.z;
            }

            origin_.x = sum_x / position_buffer_.size();
            origin_.y = sum_y / position_buffer_.size();
            origin_.z = sum_z / position_buffer_.size();
            origin_set_ = true;

            RCLCPP_INFO(this->get_logger(), "Origin set at: x=%.2f, y=%.2f, z=%.2f", 
                        origin_.x, origin_.y, origin_.z);
        }

        // Publish zeroed position
        geometry_msgs::msg::PoseStamped zeroed_msg;
        zeroed_msg.header.stamp = this->get_clock()->now();
        zeroed_msg.header.frame_id = "map";

        zeroed_msg.pose.position.x = msg->pose.pose.position.x - origin_.x;
        zeroed_msg.pose.position.y = msg->pose.pose.position.y - origin_.y;
        zeroed_msg.pose.position.z = msg->pose.pose.position.z - origin_.z;
        zeroed_msg.pose.orientation = msg->pose.pose.orientation;

        zeroed_position_pub_->publish(zeroed_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalZeroingNode>());
    rclcpp::shutdown();
    return 0;
}
