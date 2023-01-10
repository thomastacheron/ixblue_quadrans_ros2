#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ImuToPoseNode : public rclcpp::Node
{
public:
  ImuToPoseNode() : Node("imu_to_pose_node")
  {
    // Souscription au topic "imu"
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&ImuToPoseNode::imu_callback, this, std::placeholders::_1));

    // Publication sur le topic "pose"
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Création du message de type PoseStamped
    auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();

    pose_msg->header.frame_id="map";

    // Mise à jour de la position (0, 0, 0)
    pose_msg->pose.position.x = 0;
    pose_msg->pose.position.y = 0;
    pose_msg->pose.position.z = 0;

    // Mise à jour de l'orientation avec celle du message Imu
    pose_msg->pose.orientation = msg->orientation;

    // Publication du message
    pose_pub_->publish(std::move(pose_msg));
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuToPoseNode>());
  rclcpp::shutdown();
  return 0;
}
