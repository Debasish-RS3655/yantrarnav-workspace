#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdomToVisionPose : public rclcpp::Node
{
public:
    OdomToVisionPose() : Node("odom_to_vision_pose")
    {
        // Subscriber to /rtabmap/odom
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rtabmap/odom", 10,
            std::bind(&OdomToVisionPose::odom_callback, this, std::placeholders::_1));

        // Publisher to /mavros/vision_pose/pose
        vision_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/vision_pose/pose", 10);

        // Define the transform from camera_link to base_link
        tf2::Quaternion rotation;
        rotation.setRPY(0.5965, 0.5965, 0.3802);
        camera_to_base_transform_.setRotation(rotation);
        camera_to_base_transform_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));

        RCLCPP_INFO(this->get_logger(), "Odom to Vision Pose node started");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /rtabmap/odom");
        RCLCPP_INFO(this->get_logger(), "Publishing to /mavros/vision_pose/pose");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Validate the odometry message
        if (!std::isfinite(msg->pose.pose.position.x) ||
            !std::isfinite(msg->pose.pose.position.y) ||
            !std::isfinite(msg->pose.pose.position.z))
        {
            RCLCPP_WARN(this->get_logger(), "Received invalid odometry position, skipping");
            return;
        }

        // Convert odometry pose to a TF2 transform (camera_link frame)
        tf2::Transform tf2_pose_in;
        tf2::fromMsg(msg->pose.pose, tf2_pose_in);

        // Apply the transform from camera_link to base_link
        tf2::Transform pose_in_base = camera_to_base_transform_ * tf2_pose_in;
        RCLCPP_INFO(this->get_logger(), "Transformed to base_link: position (x: %f, y: %f, z: %f)",
                    pose_in_base.getOrigin().x(),
                    pose_in_base.getOrigin().y(),
                    pose_in_base.getOrigin().z());

        // Convert to geometry_msgs::Pose (still in ENU)
        geometry_msgs::msg::Pose pose_out;
        pose_out.position.x = pose_in_base.getOrigin().x();
        pose_out.position.y = pose_in_base.getOrigin().y();
        pose_out.position.z = pose_in_base.getOrigin().z();
        pose_out.orientation = tf2::toMsg(pose_in_base.getRotation());

        // Convert from ENU to NED
        geometry_msgs::msg::Pose pose_ned;
        pose_ned.position.x = pose_out.position.y;  // East -> North
        pose_ned.position.y = pose_out.position.x;  // North -> East
        pose_ned.position.z = -pose_out.position.z; // Up -> Down

        // Correct orientation conversion
        tf2::Quaternion q_enu, q_ned;
        tf2::fromMsg(pose_out.orientation, q_enu);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q_enu).getRPY(roll, pitch, yaw);
        q_ned.setRPY(roll, -pitch, -yaw);

        pose_ned.orientation = tf2::toMsg(q_ned);

        RCLCPP_INFO(this->get_logger(), "Converted to NED: position (x: %f, y: %f, z: %f)",
                    pose_ned.position.x,
                    pose_ned.position.y,
                    pose_ned.position.z);

        // Create PoseStamped message with the NED frame pose
        geometry_msgs::msg::PoseStamped vision_pose;
        vision_pose.header = msg->header;
        vision_pose.header.frame_id = "map"; // Set frame_id to map for MAVROS
        vision_pose.pose = pose_ned;

        // Publish the transformed pose
        vision_pose_pub_->publish(vision_pose);
        RCLCPP_INFO(this->get_logger(), "Published vision pose: position (x: %f, y: %f, z: %f)",
                    vision_pose.pose.position.x,
                    vision_pose.pose.position.y,
                    vision_pose.pose.position.z);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub_;
    tf2::Transform camera_to_base_transform_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToVisionPose>());
    rclcpp::shutdown();
    return 0;
}
