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
    };

    /// 创建目录，写启动标记
    bool Init(const Options& options);

    /// 每次 LIO 更新后调用
    void AddState(const NavState& state);

    /// 写一行状态，fields 为逗号分隔的各列（不含时间）
    void AddStatus(const std::string& fields);

    static const char* StatusHeader();

   private:
    void OpenTraj(const std::string& date);
    void RemoveOldTraj();

    Options options_;
    std::ofstream status_, stops_, traj_;
    std::string traj_date_;

    double last_traj_time_ = 0;
    std::deque<std::pair<double, Vec3d>> recent_;  // 最近约 1 秒的位置，用来算速度
    Vec3d last_pos_ = Vec3d::Zero();
    bool has_last_ = false;
    double path_since_stop_ = 0;
    double still_since_ = -1;
    bool stop_logged_ = true;  // 启动时还没走过，不记停靠
};

}  // namespace lightning

#endif  // LIGHTNING_LIO_RECORDER_H
