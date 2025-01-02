#include "odom_merger/odom_merger.hpp"

namespace odom_merger
{
    OdomMerger::OdomMerger(const rclcpp::NodeOptions& option): Node("OdomMerger", option)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/current",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&OdomMerger::topic_callback, this, _1)
        );

        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/merged/current", rclcpp::SystemDefaultsQoS());

        this->declare_parameter("frame_id", "map");
        this->get_parameter("frame_id", frame_id);
        this->declare_parameter("child_frame_id", "base_link");
        this->get_parameter("child_frame_id", child_frame_id);

        RCLCPP_INFO(this->get_logger(), "Start OdomMerger");
    }

    void OdomMerger::topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        try
        {
            geometry_msgs::msg::PoseStamped base_pose;
            geometry_msgs::msg::TransformStamped t;

            tf_buffer_->transform(*msg, base_pose, "base_link");

            base_pose.header.frame_id = "base_link";
            t.header.frame_id = frame_id;
            t.child_frame_id = child_frame_id;
            t.transform.rotation = base_pose.pose.orientation;
            t.transform.translation.x = base_pose.pose.position.x;
            t.transform.translation.y = base_pose.pose.position.y;
            t.transform.translation.z = base_pose.pose.position.z;

            pub_->publish(base_pose);
            tf_broadcaster_->sendTransform(t);
        }
        catch(const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform pose: %s", ex.what());
        }
        
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(odom_merger::OdomMerger)