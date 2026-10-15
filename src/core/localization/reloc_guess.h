#pragma once

#include <string>

#include "common/eigen_types.h"

namespace lightning::loc {

/**
 * 重定位初值：对机器人在地图系下位姿的猜测，对应设置它时机器人所在的位置
 * 可以来自配置文件、手动指定，或者 ScanContext、ICP、GNSS 等模块
 * 激光定位以它为初值做地图匹配，得分超过 min_init_confidence 才采用
 * 失败后过一段时间或移动后再试（朝向可信时用 LO 递推到当前位置），多次失败则丢弃
 */
struct RelocGuess {
    RelocGuess() = default;
    RelocGuess(const SE3& pose, const std::string& source, bool search_yaw = false, bool rp_from_gravity = false)
        : pose_(pose), source_(source), search_yaw_(search_yaw), rp_from_gravity_(rp_from_gravity) {}

    SE3 pose_;                 // 地图系下的位姿猜测
    std::string source_;       // 来源，仅用于日志
    bool search_yaw_ = false;  // 朝向不可信时置 true，以 pose_ 的 yaw 为中心按 grid_search_angle_* 网格搜索

    /// 置 true 时只用 pose_ 的 xyz 和 yaw，roll/pitch 取自重力对齐的 LO 姿态
    /// 要求 LIO 打开 gravity_align，且地图也是重力对齐建的
    bool rp_from_gravity_ = false;
};

}  // namespace lightning::loc
