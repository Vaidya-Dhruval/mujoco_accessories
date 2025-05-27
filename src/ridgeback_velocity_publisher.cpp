// ridgeback_example.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class RidgebackVelocityPublisher : public rclcpp::Node
{
public:
    RidgebackVelocityPublisher()
    : Node("ridgeback_velocity_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/ridgeback_velocity_controller/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RidgebackVelocityPublisher::publish_velocity, this));
    }

private:
    void publish_velocity()
    {
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";

        msg.twist.linear.x = 5.5;
        msg.twist.linear.y = 0.0;
        msg.twist.linear.z = 0.0;
        msg.twist.angular.x = 0.0;
        msg.twist.angular.y = 0.0;
        msg.twist.angular.z = 5.0;

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published velocity: linear.x = 5.5 and angular.z = 5.0");
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RidgebackVelocityPublisher>());
    rclcpp::shutdown();
    return 0;
}
