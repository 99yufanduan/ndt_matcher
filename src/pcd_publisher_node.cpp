#include <memory>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>

class PCDPublisher : public rclcpp::Node
{
public:
    PCDPublisher()
        : Node("pcd_publisher")
    {
        // 声明参数，允许用户通过命令行或配置文件传递参数
        this->declare_parameter<std::string>("pcd_file", "/home/dyf/rosbag_0816_imu_wheel_vanjee/map_tf.pcd");

        // 读取参数
        this->get_parameter("pcd_file", pcd_file_);

        // 创建发布器
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_map", 10);

        // 加载并发布 PCD 文件
        load_and_publish_pcd(pcd_file_);
    }

private:
    void load_and_publish_pcd(const std::string &pcd_file)
    {
        // 创建 PCL 点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

        // 从 PCD 文件加载点云
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", pcd_file.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded PCD file: %s with %zu points", pcd_file.c_str(), cloud->points.size());

        // 将 PCL 点云转换为 ROS 消息
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);

        // 设置帧 ID 和时间戳
        output.header.frame_id = "odom";
        output.header.stamp = this->now();

        // 发布点云消息
        publisher_->publish(output);
        RCLCPP_INFO(this->get_logger(), "Published point cloud");
    }

    std::string pcd_file_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDPublisher>());
    rclcpp::shutdown();
    return 0;
}
