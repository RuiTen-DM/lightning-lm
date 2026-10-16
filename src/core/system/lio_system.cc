//
// 持续运行的激光惯性里程计
//

#include "core/system/lio_system.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include <sys/resource.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "core/lio/laser_mapping.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"

namespace lightning {

namespace {

/// 当前进程的常驻内存（MB）
double ReadRssMB() {
    std::ifstream fin("/proc/self/status");
    std::string line;
    while (std::getline(fin, line)) {
        if (line.rfind("VmRSS:", 0) == 0) {
            return std::stod(line.substr(6)) / 1024.0;
        }
    }
    return 0;
}

builtin_interfaces::msg::Time ToRosTime(double t) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(std::floor(t));
    stamp.nanosec = static_cast<uint32_t>(std::round((t - std::floor(t)) * 1e9));
    if (stamp.nanosec >= 1000000000u) {
        stamp.sec += 1;
        stamp.nanosec -= 1000000000u;
    }
    return stamp;
}

}  // namespace

LioSystem::LioSystem() = default;

LioSystem::~LioSystem() = default;

bool LioSystem::Init(const std::string &yaml_path) {
    YAML_IO yaml(yaml_path);

    /// 可选参数，缺省用 Options 里的默认值
    const std::string sec = "lio_online";
    if (yaml.HasValue(sec, "odom_topic")) options_.odom_topic_ = yaml.GetValue<std::string>(sec, "odom_topic");
    if (yaml.HasValue(sec, "odom_frame")) options_.odom_frame_ = yaml.GetValue<std::string>(sec, "odom_frame");
    if (yaml.HasValue(sec, "base_frame")) options_.base_frame_ = yaml.GetValue<std::string>(sec, "base_frame");
    if (yaml.HasValue(sec, "pub_tf")) options_.pub_tf_ = yaml.GetValue<bool>(sec, "pub_tf");
    if (yaml.HasValue(sec, "max_extrapolation")) {
        options_.max_extrapolation_ = yaml.GetValue<double>(sec, "max_extrapolation");
    }
    if (yaml.HasValue(sec, "status_interval")) {
        options_.status_interval_ = yaml.GetValue<double>(sec, "status_interval");
    }
    auto &rec = options_.record_;
    if (yaml.HasValue(sec, "record_dir")) rec.dir_ = yaml.GetValue<std::string>(sec, "record_dir");
    if (yaml.HasValue(sec, "record_interval")) rec.interval_ = yaml.GetValue<double>(sec, "record_interval");
    if (yaml.HasValue(sec, "record_static_interval")) {
        rec.static_interval_ = yaml.GetValue<double>(sec, "record_static_interval");
    }
    if (yaml.HasValue(sec, "record_keep_days")) rec.keep_days_ = yaml.GetValue<int>(sec, "record_keep_days");

    if (!rec.dir_.empty()) {
        recorder_ = std::make_shared<LioRecorder>();
        if (!recorder_->Init(rec)) {
            LOG(ERROR) << "failed to init recorder in " << rec.dir_;
            return false;
        }
    }

    LaserMapping::Options lio_options;
    lio_options.is_in_slam_mode_ = false;  // 不保留历史关键帧
    lio_ = std::make_shared<LaserMapping>(lio_options);
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio";
        return false;
    }

    node_ = std::make_shared<rclcpp::Node>("lightning_lio");

    const auto imu_topic = yaml.GetValue<std::string>("common", "imu_topic");
    const auto cloud_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    const auto livox_topic = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    // IMU 一条都不能丢，队列要能装下 LIO 处理一帧期间到达的 IMU
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::QoS(500), [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            ProcessIMU(imu);
        });

    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::QoS(5), [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            lio_->ProcessPointCloud2(msg);
            AfterLidar();
        });

    livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic, rclcpp::QoS(5), [this](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
            lio_->ProcessPointCloud2(msg);
            AfterLidar();
        });

    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(options_.odom_topic_, rclcpp::QoS(100));
    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    }

    timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() { OnTimer(); });

    RCLCPP_INFO(node_->get_logger(), "lio online: imu %s, lidar %s / %s -> %s (%s -> %s), tf %d", imu_topic.c_str(),
                cloud_topic.c_str(), livox_topic.c_str(), options_.odom_topic_.c_str(), options_.odom_frame_.c_str(),
                options_.base_frame_.c_str(), options_.pub_tf_);
    return true;
}

void LioSystem::Spin() {
    if (node_ != nullptr) {
        rclcpp::spin(node_);
    }
}

void LioSystem::ProcessIMU(const IMUPtr &imu) {
    num_imu_++;
    lio_->ProcessIMU(imu);

    if (scan_waiting_) {
        RunLIO();
    }

    // 第一帧点云只用来建局部地图，从第二帧起 IMU 外推的状态才有效
    if (num_frames_ < 2) {
        return;
    }

    if (imu->timestamp - last_lio_time_ > options_.max_extrapolation_) {
        if (!extrapolation_warned_) {
            LOG(WARNING) << "no lidar update for " << imu->timestamp - last_lio_time_ << " s, stop publishing odom";
            extrapolation_warned_ = true;
        }
        return;
    }
    extrapolation_warned_ = false;

    Publish(lio_->GetIMUState(), imu->angular_velocity, imu->timestamp);
}

void LioSystem::AfterLidar() {
    num_scans_++;
    num_dropped_ += lio_->LimitBuffers(options_.max_imu_age_, options_.max_pending_scans_);
    RunLIO();
}

void LioSystem::RunLIO() {
    while (lio_->HasPendingScan()) {
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = lio_->Run();
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        if (!ok) {
            // 多数是 IMU 还没覆盖到这一帧的结束时间，等后续 IMU 到了再处理；空点云等情况这一帧已被丢掉
            scan_waiting_ = lio_->HasPendingScan();
            return;
        }

        num_frames_++;
        interval_frames_++;
        interval_run_ms_ += ms;
        interval_max_ms_ = std::max(interval_max_ms_, ms);
        last_lio_time_ = lio_->GetState().timestamp_;
        if (recorder_ && num_frames_ >= 2) {
            recorder_->AddState(lio_->GetState());
        }
    }

    scan_waiting_ = false;
}

void LioSystem::Publish(const NavState &state, const Vec3d &gyro, double stamp) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = ToRosTime(stamp);
    odom.header.frame_id = options_.odom_frame_;
    odom.child_frame_id = options_.base_frame_;

    const Vec3d p = state.pos_;
    const Quatd q = state.rot_.unit_quaternion();
    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = p.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // twist 在 base_frame 下
    const Vec3d v = state.rot_.inverse() * state.vel_;
    const Vec3d w = gyro - state.bg_;
    odom.twist.twist.linear.x = v.x();
    odom.twist.twist.linear.y = v.y();
    odom.twist.twist.linear.z = v.z();
    odom.twist.twist.angular.x = w.x();
    odom.twist.twist.angular.y = w.y();
    odom.twist.twist.angular.z = w.z();

    odom_pub_->publish(odom);
    num_published_++;

    if (tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header = odom.header;
        tf.child_frame_id = options_.base_frame_;
        tf.transform.translation.x = p.x();
        tf.transform.translation.y = p.y();
        tf.transform.translation.z = p.z();
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }
}

void LioSystem::OnTimer() {
    // 雷达停了的时候点云回调不再触发，在这里也限制一下缓存
    num_dropped_ += lio_->LimitBuffers(options_.max_imu_age_, options_.max_pending_scans_);

    if (options_.status_interval_ <= 0) {
        return;
    }

    const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (last_status_time_ > 0 && now - last_status_time_ < options_.status_interval_) {
        return;
    }

    const double interval = now - last_status_time_;
    last_status_time_ = now;

    // 进程在这段时间里的 CPU 占用（占单核的百分比）
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    const double cpu_time = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec * 1e-6 + usage.ru_stime.tv_sec +
                            usage.ru_stime.tv_usec * 1e-6;
    const double cpu_pct = last_cpu_time_ > 0 && interval > 0 ? (cpu_time - last_cpu_time_) / interval * 100 : 0;
    last_cpu_time_ = cpu_time;

    const auto state = lio_->GetState();
    const double avg_ms = interval_frames_ > 0 ? interval_run_ms_ / interval_frames_ : 0;
    const double rss = ReadRssMB();
    RCLCPP_INFO(node_->get_logger(),
                "frames %ld/%ld scans, dropped %ld, imu %ld, odom %ld | lio %.2f ms avg, %.2f ms max | "
                "local map %zu grids | rss %.1f MB | cpu %.0f%% | pos %.2f %.2f %.2f",
                num_frames_, num_scans_, num_dropped_, num_imu_, num_published_, avg_ms, interval_max_ms_,
                lio_->NumLocalMapGrids(), rss, cpu_pct, state.pos_.x(), state.pos_.y(), state.pos_.z());

    if (recorder_) {
        const Mat3d R = state.rot_.matrix();
        char fields[256];
        snprintf(fields, sizeof(fields), "%ld,%ld,%ld,%.2f,%.2f,%zu,%.1f,%.0f,%.3f,%.3f,%.3f,%.2f", num_frames_,
                 num_scans_, num_dropped_, avg_ms, interval_max_ms_, lio_->NumLocalMapGrids(), rss, cpu_pct,
                 state.pos_.x(), state.pos_.y(), state.pos_.z(), std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI);
        recorder_->AddStatus(fields);
    }

    interval_frames_ = 0;
    interval_run_ms_ = 0;
    interval_max_ms_ = 0;
}

}  // namespace lightning
