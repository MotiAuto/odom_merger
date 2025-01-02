#ifndef ODOM_MERGER_HPP_
#define ODOM_MERGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>

using std::placeholders::_1;

namespace odom_merger
{
    class OdomMerger : public rclcpp::Node
    {
        public:
        explicit OdomMerger(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());

        void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        private:
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
        std::string frame_id, child_frame_id;
    };
}

#endif