//
// 里程计结果的精简记录
//

#include "core/system/lio_recorder.h"

#include <glog/logging.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>

namespace lightning {

namespace {

double WallTime() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

/// 本地时间，fmt 为 strftime 格式
std::string LocalTime(double t, const char* fmt) {
    time_t sec = static_cast<time_t>(t);
    struct tm tm_local;
    localtime_r(&sec, &tm_local);
    char buf[32];
    strftime(buf, sizeof(buf), fmt, &tm_local);
    return buf;
}

/// 以追加方式打开，新文件先写表头
void OpenCsv(std::ofstream& f, const std::string& path, const char* header) {
    const bool is_new = !std::filesystem::exists(path) || std::filesystem::file_size(path) == 0;
    f.open(path, std::ios::app);
    if (is_new) {
        f << header << "\n";
    }
}

double Yaw(const NavState& s) {
    const Mat3d R = s.rot_.matrix();
    return std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
}

}  // namespace

const char* LioRecorder::StatusHeader() {
    return "time,frames,scans,dropped,lio_ms_avg,lio_ms_max,grids,rss_mb,cpu_pct,x,y,z,yaw_deg";
}

bool LioRecorder::Init(const Options& options) {
    options_ = options;
    std::error_code ec;
    std::filesystem::create_directories(options_.dir_, ec);
    if (ec) {
        LOG(ERROR) << "failed to create record dir " << options_.dir_ << ": " << ec.message();
        return false;
    }

    OpenCsv(status_, options_.dir_ + "/status.csv", StatusHeader());
    OpenCsv(stops_, options_.dir_ + "/stops.csv", "time,x,y,z,yaw_deg,path_m");
    status_ << "# start " << LocalTime(WallTime(), "%F %T") << " pid " << getpid() << std::endl;

    OpenTraj(LocalTime(WallTime(), "%Y%m%d"));
    return status_.good() && stops_.good() && traj_.good();
}

void LioRecorder::OpenTraj(const std::string& date) {
    if (traj_.is_open()) {
        traj_.close();
    }
    traj_date_ = date;
    OpenCsv(traj_, options_.dir_ + "/traj_" + date + ".csv", "time,x,y,z,yaw_deg");
    RemoveOldTraj();
}

void LioRecorder::RemoveOldTraj() {
    if (options_.keep_days_ <= 0) {
        return;
    }
    const std::string oldest = LocalTime(WallTime() - options_.keep_days_ * 86400.0, "%Y%m%d");
    for (const auto& entry : std::filesystem::directory_iterator(options_.dir_)) {
        const std::string name = entry.path().filename().string();
        // traj_YYYYMMDD.csv，按文件名里的日期判断
        if (name.size() == 18 && name.rfind("traj_", 0) == 0 && name.substr(5, 8) < oldest) {
            std::error_code ec;
            std::filesystem::remove(entry.path(), ec);
        }
    }
}

void LioRecorder::AddState(const NavState& state) {
    const double now = WallTime();
    const Vec3d& p = state.pos_;

    // 速度用最近约 1 秒的位移来算，比滤波器里的速度稳
    recent_.emplace_back(now, p);
    while (recent_.size() > 2 && now - recent_.front().first > 1.0) {
        recent_.pop_front();
    }
    const double dt = now - recent_.front().first;
    const double speed = dt > 0.5 ? (p - recent_.front().second).norm() / dt : 0;
    const bool moving = speed > options_.stop_speed_;

    // 只在移动时累计路程，静止时滤波器的抖动累积起来也不小
    if (has_last_ && moving) {
        path_since_stop_ += (p - last_pos_).norm();
    }
    last_pos_ = p;
    has_last_ = true;

    // 轨迹
    const std::string date = LocalTime(now, "%Y%m%d");
    if (date != traj_date_) {
        OpenTraj(date);
    }
    if (now - last_traj_time_ >= (moving ? options_.interval_ : options_.static_interval_)) {
        last_traj_time_ = now;
        char line[128];
        snprintf(line, sizeof(line), "%.1f,%.3f,%.3f,%.3f,%.2f", now, p.x(), p.y(), p.z(), Yaw(state));
        traj_ << line << std::endl;
    }

    // 停靠点
    if (moving) {
        still_since_ = -1;
        stop_logged_ = false;
        return;
    }
    if (still_since_ < 0) {
        still_since_ = now;
    }
    if (!stop_logged_ && now - still_since_ >= options_.stop_time_ && path_since_stop_ >= options_.min_path_) {
        char line[160];
        snprintf(line, sizeof(line), "%s,%.3f,%.3f,%.3f,%.2f,%.1f", LocalTime(now, "%F %T").c_str(), p.x(), p.y(),
                 p.z(), Yaw(state), path_since_stop_);
        stops_ << line << std::endl;
        stop_logged_ = true;
        path_since_stop_ = 0;
    }
}

void LioRecorder::AddStatus(const std::string& fields) {
    status_ << LocalTime(WallTime(), "%F %T") << "," << fields << std::endl;
}

}  // namespace lightning
