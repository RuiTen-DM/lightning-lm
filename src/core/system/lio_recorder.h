//
// 里程计结果的精简记录，供长期运行后取回查看
//

#ifndef LIGHTNING_LIO_RECORDER_H
#define LIGHTNING_LIO_RECORDER_H

#include <deque>
#include <fstream>
#include <string>

#include "common/eigen_types.h"
#include "common/nav_state.h"

namespace lightning {

/**
 * 在 dir 下写三种 csv，时间都用系统时间：
 *  - status.csv     运行状态，每次 AddStatus 一行；程序每次启动先写一行 "# start ..."，可以看出重启
 *  - stops.csv      每次停稳（移动至少 min_path 米后，静止 stop_time 秒）记一行位姿和这一段走过的路程。
 *                   往复运动时，同一停靠点的位置随时间的变化就是里程计的漂移
 *  - traj_日期.csv  轨迹：移动时每 interval 秒一行，静止时每 static_interval 秒一行；按天分文件，超过 keep_days 天的删掉
 *  - still.csv      静止时的漂移：由 IMU 判断真正静止（不靠里程计自己），记下静止期间里程计位置变了多少。
 *                   静止至少 min_still 秒才记；静止很久时每 still_report 秒记一行累计值，结束时再记一行（final=1）
 */
class LioRecorder {
   public:
    struct Options {
        std::string dir_;
        double interval_ = 1.0;
        double static_interval_ = 60.0;
        int keep_days_ = 30;
        double stop_speed_ = 0.05;  // 低于这个速度（m/s）算静止
        double stop_time_ = 3.0;    // 静止多久算停稳（秒）
        double min_path_ = 1.0;     // 两次停稳之间至少走过多少米才记
        double min_still_ = 30.0;      // IMU 判断静止至少多久才记静止漂移（秒）
        double still_report_ = 600.0;  // 长时间静止时，每隔多久记一行累计漂移（秒）
    };

    /// 创建目录，写启动标记
    bool Init(const Options& options);

    /// 每次 LIO 更新后调用，imu_still 为 IMU 判断的是否静止
    void AddState(const NavState& state, bool imu_still);

    /// 退出前调用：正在进行的静止段如果够长，也记下来
    void Finish();

    /// 写一行状态，fields 为逗号分隔的各列（不含时间）
    void AddStatus(const std::string& fields);

    static const char* StatusHeader();

   private:
    void OpenTraj(const std::string& date);
    void UpdateStill(const NavState& state, bool imu_still, double now);
    void WriteStill(const NavState& state, double now, bool final);
    void RemoveOldTraj();

    Options options_;
    std::ofstream status_, stops_, traj_, still_;
    std::string traj_date_;

    double last_traj_time_ = 0;
    std::deque<std::pair<double, Vec3d>> recent_;  // 最近约 1 秒的位置，用来算速度
    Vec3d last_pos_ = Vec3d::Zero();
    bool has_last_ = false;
    double path_since_stop_ = 0;
    double still_since_ = -1;
    bool stop_logged_ = true;  // 启动时还没走过，不记停靠

    /// 当前这段静止的起点
    bool in_still_ = false;
    double still_start_ = 0;
    double last_still_report_ = 0;
    Vec3d still_pos_ = Vec3d::Zero();
    double still_yaw_ = 0;
    NavState last_state_;
    bool has_state_ = false;
};

}  // namespace lightning

#endif  // LIGHTNING_LIO_RECORDER_H
