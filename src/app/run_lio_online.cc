//
// 持续运行的激光惯性里程计（只跑 LIO，不建图、不用地图）
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/system/lio_system.h"

DEFINE_string(config, "./config/default_livox.yaml", "配置文件");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    // 长期运行：LIO 每帧都打好几行 INFO，默认只记 WARNING 及以上，避免日志写满磁盘
    // 需要逐帧日志时加 --minloglevel=0；运行状态由 ros 日志定期打印（lio_online/status_interval）
    FLAGS_minloglevel = google::WARNING;
    google::ParseCommandLineFlags(&argc, &argv, true);

    rclcpp::init(argc, argv);

    lightning::LioSystem lio;
    if (!lio.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init lio";
        rclcpp::shutdown();
        return -1;
    }

    lio.Spin();
    rclcpp::shutdown();
    return 0;
}
