#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class TFBroadcaster : public rclcpp::Node
{
public:
    TFBroadcaster()
    : Node("tf_broadcaster")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&TFBroadcaster::broadcast_transform, this));
    }

private:
    void broadcast_transform()
    {
        geometry_msgs::msg::TransformStamped t;

        auto now = this->get_clock()->now();
        double time_sec = now.seconds();

        t.header.stamp = now;
        t.header.frame_id = "world";
        t.child_frame_id = "robot";

        // Circular motion
        t.transform.translation.x = 2.0 * std::cos(time_sec);
        t.transform.translation.y = 2.0 * std::sin(time_sec);
        t.transform.translation.z = 0.0;

        // Identity quaternion (no rotation)
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
