//
// 持续运行的激光惯性里程计
//

#ifndef LIGHTNING_LIO_SYSTEM_H
#define LIGHTNING_LIO_SYSTEM_H

#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/nav_state.h"

namespace lightning {

class LaserMapping;

/**
 * 持续运行的激光惯性里程计：只跑 LIO，不建图，也不用地图
 *
 * 为长期运行设计：不保留历史关键帧，局部地图（iVox）有格子数上限（fasterlio/ivox_capacity），
 * 输入缓存也有上限，内存不随运行时间增长
 *
 * 输出 odom_frame -> base_frame 的位姿（base_frame 即 IMU 系），IMU 频率：
 *  - nav_msgs/Odometry，twist 在 base_frame 下
 *  - TF（可关）
 * 两次激光更新之间按匀速和陀螺外推；超过 max_extrapolation 秒没有激光更新就停发，避免外推跑飞
 *
 * 所有回调都在同一个单线程 executor 里执行，LIO 不需要加锁
 */
class LioSystem {
   public:
    struct Options {
        std::string odom_topic_ = "lightning/odom";
        std::string odom_frame_ = "odom";
        std::string base_frame_ = "base_link";
        bool pub_tf_ = true;
        double max_extrapolation_ = 0.5;  // 距上次激光更新超过这么久（秒）就不再发布
        double status_interval_ = 10.0;   // 状态日志的间隔（秒），<= 0 不打印
        double max_imu_age_ = 2.0;        // 缓存里最多保留多久以前的 IMU（秒）
        int max_pending_scans_ = 10;      // 最多缓存多少帧还没处理的点云（IMU 断流时会积压），多出的丢掉
    };

    LioSystem();
    ~LioSystem();

    /// 读取配置、创建 LIO 和 ros2 节点
    bool Init(const std::string& yaml_path);

    /// 阻塞运行，直到 ctrl-c
    void Spin();

   private:
    void ProcessIMU(const IMUPtr& imu);
    void AfterLidar();

    /// 处理所有已缓存、且 IMU 已经覆盖到的点云
    void RunLIO();

    void Publish(const NavState& state, const Vec3d& gyro, double stamp);

    /// 定时：限制缓存，打印状态
    void OnTimer();

    Options options_;
    std::shared_ptr<LaserMapping> lio_ = nullptr;

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_ = nullptr;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_ = nullptr;
    rclcpp::TimerBase::SharedPtr timer_ = nullptr;

    bool scan_waiting_ = false;  // 有点云在等后续的 IMU
    double last_lio_time_ = 0;   // 最近一次激光更新的时间
    bool extrapolation_warned_ = false;

    /// 统计，状态日志里打印后清零的是区间值
    int64_t num_imu_ = 0;
    int64_t num_scans_ = 0;      // 收到的点云
    int64_t num_frames_ = 0;     // LIO 成功处理的帧
    int64_t num_dropped_ = 0;    // 因缓存超限丢掉的点云
    int64_t num_published_ = 0;  // 发布的 odom
    double interval_run_ms_ = 0, interval_max_ms_ = 0;
    int64_t interval_frames_ = 0;
    double last_status_time_ = 0;
};

}  // namespace lightning

#endif  // LIGHTNING_LIO_SYSTEM_H
