#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class LidarImuTF2Publisher : public rclcpp::Node
{
public:
    LidarImuTF2Publisher() : Node("lidar_imu_tf2_publisher")
    {
        // Initialize the TransformBroadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Subscribe to the LiDAR and IMU sensor topics
        lidar_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LidarImuTF2Publisher::lidarCallback, this, std::placeholders::_1));

        imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu_data", 10, std::bind(&LidarImuTF2Publisher::imuCallback, this, std::placeholders::_1));
    }

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg)
    {
        // Assuming the LiDAR frame is "lidar_frame"
        geometry_msgs::msg::TransformStamped lidar_tf;
        lidar_tf.header.stamp = now();
        lidar_tf.header.frame_id = "world_frame"; // The parent frame (the fixed reference frame)
        lidar_tf.child_frame_id = lidar_msg->header.frame_id;  // The child frame (the LiDAR frame)

        // Set the translation and rotation values based on LiDAR pose
        lidar_tf.transform.translation.x = 1.0; // Replace with actual X translation
        lidar_tf.transform.translation.y = 2.0; // Replace with actual Y translation
        lidar_tf.transform.translation.z = 0.0; // Replace with actual Z translation

        // Assuming the LiDAR orientation is identity (no rotation relative to parent frame)
        lidar_tf.transform.rotation.x = 0.0;
        lidar_tf.transform.rotation.y = 0.0;
        lidar_tf.transform.rotation.z = 0.0;
        lidar_tf.transform.rotation.w = 1.0;

        // Publish the LiDAR transformation
        tf_broadcaster_->sendTransform(lidar_tf);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        // Assuming the IMU frame is "imu_frame"
        geometry_msgs::msg::TransformStamped imu_tf;
        imu_tf.header.stamp = now();
        imu_tf.header.frame_id = "world_frame"; // The parent frame (the fixed reference frame)
        imu_tf.child_frame_id = imu_msg->header.frame_id;    // The child frame (the IMU frame)

        // Set the translation and rotation values based on IMU pose
        imu_tf.transform.translation.x = 0.5; // Replace with actual X translation
        imu_tf.transform.translation.y = 0.5; // Replace with actual Y translation
        imu_tf.transform.translation.z = 0.0; // Replace with actual Z translation

        // Convert the IMU orientation to a quaternion and set the rotation
        tf2::Quaternion q;
        tf2::fromMsg(imu_msg->orientation, q);
        imu_tf.transform.rotation = tf2::toMsg(q);

        // Publish the IMU transformation
        tf_broadcaster_->sendTransform(imu_tf);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarImuTF2Publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
