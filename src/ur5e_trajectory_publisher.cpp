#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class UR5eTrajectoryPublisher : public rclcpp::Node
{
public:
    UR5eTrajectoryPublisher()
    : Node("ur5e_trajectory_publisher")
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&UR5eTrajectoryPublisher::publish_trajectory, this));
    }

private:
    void publish_trajectory()
    {
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {0.0, -1.57, 1.57, -1.57, 1.57, 0.0};
        point.time_from_start = rclcpp::Duration::from_seconds(3.0);

        traj_msg.points.push_back(point);

        publisher_->publish(traj_msg);
        RCLCPP_INFO(this->get_logger(), "Sent UR5e trajectory point.");

        timer_->cancel();  // send only once
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UR5eTrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
