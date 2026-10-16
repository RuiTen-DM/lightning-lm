//
// 把一整张点云地图（如手工裁剪过的 pcd）转成定位用的分块地图目录
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <fstream>
#include <sstream>

#include "common/constant.h"
#include "core/lightning_math.hpp"
#include "core/maps/tiled_map.h"

DEFINE_string(input_pcd, "", "输入的整张点云地图，坐标在地图系下");
DEFINE_string(output_dir, "", "输出的地图目录（定位时的 map_path），须不存在或为空");
DEFINE_string(start_from, "", "从这个地图目录的 index.txt 拷贝功能点 start（建图起点的位姿），两张地图须在同一坐标系");
DEFINE_string(start_pose, "", "或者直接给出 start：x,y,z,roll,pitch,yaw（米、度）");

using namespace lightning;

/// 读地图目录里 index.txt 的 start 功能点
bool ReadStart(const std::string& map_dir, SE3& pose) {
    std::ifstream fin(map_dir + "/index.txt");
    std::string line;
    while (std::getline(fin, line)) {
        std::stringstream ss(line);
        std::string name;
        ss >> name;
        if (name != "start") {
            continue;
        }
        double d[7];
        for (double& v : d) {
            ss >> v;
        }
        // 与 TiledMap 的存储顺序一致：x y z qx qy qz qw
        pose = SE3(Quatd(d[6], d[3], d[4], d[5]).normalized(), Vec3d(d[0], d[1], d[2]));
        return true;
    }
    return false;
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_input_pcd.empty() || FLAGS_output_dir.empty()) {
        LOG(ERROR) << "需要 --input_pcd 和 --output_dir";
        return -1;
    }

    SE3 start;
    if (!FLAGS_start_from.empty()) {
        if (!ReadStart(FLAGS_start_from, start)) {
            LOG(ERROR) << "在 " << FLAGS_start_from << "/index.txt 里没找到 start";
            return -1;
        }
    } else if (!FLAGS_start_pose.empty()) {
        std::vector<double> v;
        std::stringstream ss(FLAGS_start_pose);
        for (std::string item; std::getline(ss, item, ',');) {
            v.push_back(std::stod(item));
        }
        if (v.size() != 6) {
            LOG(ERROR) << "--start_pose 应为 x,y,z,roll,pitch,yaw";
            return -1;
        }
        using constant::kDEG2RAD;
        start = math::XYZRPYToSE3(PoseRPYD(v[0], v[1], v[2], v[3] * kDEG2RAD, v[4] * kDEG2RAD, v[5] * kDEG2RAD));
    } else {
        LOG(WARNING) << "没有给 start，功能点 start 取地图原点、零姿态";
    }

    namespace fs = std::filesystem;
    if (fs::exists(FLAGS_output_dir) && !fs::is_empty(FLAGS_output_dir)) {
        LOG(ERROR) << "输出目录不为空: " << FLAGS_output_dir;
        return -1;
    }
    fs::create_directories(FLAGS_output_dir);

    CloudPtr cloud(new PointCloudType);
    if (pcl::io::loadPCDFile(FLAGS_input_pcd, *cloud) != 0 || cloud->empty()) {
        LOG(ERROR) << "读取失败或为空: " << FLAGS_input_pcd;
        return -1;
    }
    LOG(INFO) << "loaded " << cloud->size() << " points, start: " << start.translation().transpose() << ", q "
              << start.unit_quaternion().coeffs().transpose();

    TiledMap::Options options;
    options.map_path_ = FLAGS_output_dir;
    TiledMap tm(options);
    tm.ConvertFromFullPCD(cloud, start, FLAGS_output_dir);

    // 定位只读分块，global.pcd 留着方便查看，和建图输出的目录结构一致
    pcl::io::savePCDFileBinaryCompressed(FLAGS_output_dir + "/global.pcd", *cloud);
    LOG(INFO) << "done: " << FLAGS_output_dir;
    return 0;
}
