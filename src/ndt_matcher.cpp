#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// ros tf2 相关
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_eigen/tf2_eigen.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "ndt_msgs/msg/point_cloud_map_cell_with_id.hpp"

#include <memory>
using std::placeholders::_1;

#include <string>

#include "multigrid_pclomp/multi_voxel_grid_covariance_omp.h"
#include "multigrid_pclomp/multigrid_ndt_omp.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <mutex>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
using NormalDistributionsTransform =
    pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;

class NdtMatcher : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr pose_in);
    void pcCallback(const sensor_msgs::msg::PointCloud2::UniquePtr pc_ros_in);
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    int mapLoadAndGrid(std::string map_path);
    std::shared_ptr<NormalDistributionsTransform> ndt_ptr_;
    std::mutex *ndt_ptr_mutex_;
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg_;

public:
    NdtMatcher() : Node("imu_kalman_node"), ndt_ptr_(new pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>()), ndt_ptr_mutex_(new std::mutex())
    {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/ekf_pose_with_covariance", 1, std::bind(&NdtMatcher::poseCallback, this, _1));

        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar_points", 1, std::bind(&NdtMatcher::pcCallback, this, _1));
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ndt_pose", 10);
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_points_tf", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // 初始化 PoseWithCovarianceStamped 消息的时间戳和帧 ID
        initial_pose_msg_.header.stamp = rclcpp::Clock().now();
        initial_pose_msg_.header.frame_id = "odom"; // 可以根据需要设置适当的帧 ID

        // 初始化 PoseWithCovariance 消息
        initial_pose_msg_.pose.pose.position.x = 0.0;
        initial_pose_msg_.pose.pose.position.y = 0.0;
        initial_pose_msg_.pose.pose.position.z = 0.0;
        initial_pose_msg_.pose.pose.orientation.x = 0.0;
        initial_pose_msg_.pose.pose.orientation.y = 0.0;
        initial_pose_msg_.pose.pose.orientation.z = 0.0;
        initial_pose_msg_.pose.pose.orientation.w = 1.0; // 默认的四元数表示无旋转

        // 初始化协方差矩阵为零
        initial_pose_msg_.pose.covariance.fill(0.0);

        pclomp::NdtParams ndt_params;
        ndt_params.trans_epsilon = 0.01;
        ndt_params.step_size = 0.1;
        ndt_params.resolution = 2;
        ndt_params.max_iterations = 60;
        ndt_params.num_threads = 1;
        ndt_params.regularization_scale_factor = 0.01;
        ndt_ptr_->setParams(ndt_params);

        this->mapLoad("/home/dyf/rosbag_0816_imu_wheel_vanjee/map_tf.pcd");
    }
    void update_ndt(
        const std::vector<ndt_msgs::msg::PointCloudMapCellWithID> &maps_to_add,
        const std::vector<std::string> &map_ids_to_remove);

    int mapLoad(std::string map_path);
    void publish_pose(const rclcpp::Time &sensor_ros_time, const geometry_msgs::msg::Pose &result_pose_msg);
};

Eigen::Affine3d pose_to_affine3d(const geometry_msgs::msg::Pose &ros_pose)
{
    Eigen::Affine3d eigen_pose;
    tf2::fromMsg(ros_pose, eigen_pose);
    return eigen_pose;
}

Eigen::Matrix4f pose_to_matrix4f(const geometry_msgs::msg::Pose &ros_pose)
{
    Eigen::Affine3d eigen_pose_affine = pose_to_affine3d(ros_pose);
    Eigen::Matrix4f eigen_pose_matrix = eigen_pose_affine.matrix().cast<float>();
    return eigen_pose_matrix;
}

geometry_msgs::msg::Pose matrix4f_to_pose(const Eigen::Matrix4f &eigen_pose_matrix)
{
    Eigen::Affine3d eigen_pose_affine;
    eigen_pose_affine.matrix() = eigen_pose_matrix.cast<double>();
    geometry_msgs::msg::Pose ros_pose = tf2::toMsg(eigen_pose_affine);
    return ros_pose;
}

void NdtMatcher::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr pose_in)
{
    initial_pose_msg_ = *pose_in;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "initial pose update");
}

void NdtMatcher::pcCallback(const sensor_msgs::msg::PointCloud2::UniquePtr pc_ros_in)
{
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_in_filter(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indice;
    pcl::fromROSMsg(*pc_ros_in, *pc_in);
    pcl::removeNaNFromPointCloud(*pc_in, *pc_in_filter, indice);
    ndt_ptr_->setInputSource(pc_in_filter);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "pointcloud input done");
    auto initial_pose_matrix = pose_to_matrix4f(initial_pose_msg_.pose.pose);
    auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    ndt_ptr_->align(*output_cloud, initial_pose_matrix);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "ndt done");
    const pclomp::NdtResult ndt_result = ndt_ptr_->getResult();
    const geometry_msgs::msg::Pose result_pose_msg = matrix4f_to_pose(ndt_result.pose);
    publish_pose(pc_ros_in->header.stamp, result_pose_msg);

    // 获取变换
    geometry_msgs::msg::TransformStamped transform_stamped;
    // 从Pose创建TransformStamped
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "odom";     // 原坐标系
    transform_stamped.child_frame_id = "base_link"; // 目标坐标系
    transform_stamped.transform.translation.x = result_pose_msg.position.x;
    transform_stamped.transform.translation.y = result_pose_msg.position.y;
    transform_stamped.transform.translation.z = result_pose_msg.position.z;
    transform_stamped.transform.rotation = result_pose_msg.orientation;

    // 使用tf2进行变换
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    tf2::doTransform(*pc_ros_in, transformed_cloud, transform_stamped);

    // 发布变换后的点云
    pc_pub_->publish(transformed_cloud);
}

void NdtMatcher::publish_pose(
    const rclcpp::Time &sensor_ros_time, const geometry_msgs::msg::Pose &result_pose_msg)
{
    geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
    result_pose_with_cov_msg.header.stamp = sensor_ros_time;
    result_pose_with_cov_msg.header.frame_id = "odom";
    result_pose_with_cov_msg.pose.pose = result_pose_msg;
    result_pose_with_cov_msg.pose.covariance = {0};

    pub_->publish(result_pose_with_cov_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NdtMatcher>());
    rclcpp::shutdown();
    return 0;
}

void NdtMatcher::update_ndt(
    const std::vector<ndt_msgs::msg::PointCloudMapCellWithID> &maps_to_add,
    const std::vector<std::string> &map_ids_to_remove)
{
    RCLCPP_INFO(
        this->get_logger(), "Update map (Add: %lu, Remove: %lu)", maps_to_add.size(), map_ids_to_remove.size());
    if (maps_to_add.empty() && map_ids_to_remove.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Skip map update");
        return;
    }
    const auto exe_start_time = std::chrono::system_clock::now();

    NormalDistributionsTransform backup_ndt = *ndt_ptr_;

    // Add pcd
    for (const auto &map_to_add : maps_to_add)
    {
        pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(map_to_add.pointcloud, *map_points_ptr);
        backup_ndt.addTarget(map_points_ptr, map_to_add.cell_id);
    }

    // Remove pcd
    for (const std::string &map_id_to_remove : map_ids_to_remove)
    {
        backup_ndt.removeTarget(map_id_to_remove);
    }

    backup_ndt.createVoxelKdtree();

    const auto exe_end_time = std::chrono::system_clock::now();
    const double exe_time =
        std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() /
        1000.0;
    RCLCPP_INFO(this->get_logger(), "Time duration for creating new ndt_ptr: %lf [ms]", exe_time);

    // swap
    (*ndt_ptr_mutex_).lock();
    // ToDo (kminoda): Here negligible NDT copy occurs during the new map loading phase, which should
    // ideally be avoided. But I will leave this for now since I cannot come up with a solution other
    // than using pointer of pointer.
    *ndt_ptr_ = backup_ndt;
    (*ndt_ptr_mutex_).unlock();

    // publish_partial_pcd_map();
}

int NdtMatcher::mapLoad(std::string map_path)
{
    NormalDistributionsTransform new_ndt;
    new_ndt.setParams(ndt_ptr_->getParams());

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *map_cloud) == -1)
    {
        PCL_ERROR("Couldn't read pcd file.\n");
        return (-1);
    }
    std::cout << "12:34:19: " << *map_cloud << std::endl;
    new_ndt.setInputTarget(map_cloud);
    // swap
    ndt_ptr_mutex_->lock();
    *ndt_ptr_ = new_ndt;
    ndt_ptr_mutex_->unlock();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "map load done");
    return 0;
}