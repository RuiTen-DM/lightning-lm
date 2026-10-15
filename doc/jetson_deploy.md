# lightning-lm 在 Jetson Orin Nano 上的编译部署记录

记录时间：2026-10-14　设备：`dm@10.198.1.219`

| 项目 | 情况 |
|---|---|
| 硬件 | Jetson Orin Nano Engineering Reference Developer Kit Super，6 核，内存 7.4 GB，swap 3.7 GB，功耗模式 25W |
| 系统 | L4T R36.5，Ubuntu 22.04.5，aarch64，GCC 11.4，CMake 3.22 |
| ROS | ROS 2 Humble（设备上已预装 ros-base） |

## 结论

- **编译成功，没有改动任何源码。** 只需要加编译参数并补齐依赖。`colcon build` 用 `-j2` 耗时 19 分 49 秒。
- **四个程序都在设备上跑通了。** 测试数据是一个 8 秒的 Mid-360 包（79 帧点云、200 Hz IMU），机器人基本没动，建图只出了 1 个关键帧。所以下面的数字只能证明程序能跑，**不能当性能评估**。

| 测试 | 结果 |
|---|---|
| `run_slam_offline` | 正常出图（分块 pcd + `map.pgm`），8 秒的数据 1.3 秒跑完（含启动）。单次匹配平均 0.54 ms（x86 上 0.32 ms），去畸变 0.55 ms（x86 上 0.33 ms） |
| `run_loc_offline` | 从地图起点自动初始化成功（`init with fp: start`） |
| `run_loc_online` + `ros2 bag play` | 初始化成功，`/tf` 持续输出。回放期间占约 27% 单核，常驻内存约 72 MB，22 个线程 |
| `run_slam_online` + `ros2 bag play` | 占约 29% 单核；调 `/lightning/save_map` 服务能正常存图 |

- **Livox-SDK2 和 livox_ros_driver2 也已装好**，见下面"Livox SDK 和驱动"一节。设备目前没接雷达，只验证到驱动能启动、能读配置，以及用驱动自己的消息包回放数据、lightning 能正常接收。

## 目录布局

所有为本库下载、编译的东西都放在 `~/lightning-lm` 下：

```
~/lightning-lm/                     源码，从开发机 rsync 过来（含本地未提交的修改）
├── config/default_livox_jetson.yaml   无头运行用的配置，只在 Jetson 上有（with_ui: false）
├── data/bags/                      冒烟测试用的包
├── data/jetson_test/               冒烟测试的工作目录、日志、地图
├── build/ install/ bin/            colcon 编译产物
└── thirdparty/
    ├── Pangolin-0.9.3.zip          仓库自带的源码
    ├── deps_install/               Pangolin（静态库）和 Livox-SDK2（lib/ 和 include/）的安装位置
    └── deps_build/                 带 COLCON_IGNORE
        ├── apt/                    本库专用的 apt 配置；archives/ 里是下载的 322 个 .deb（305 MB）
        ├── src/                    Pangolin-0.9.3、Livox-SDK2（v1.3.1）的源码
        ├── pangolin_build/ livox_sdk2_build/   对应的编译目录
        ├── livox_ws/               livox_ros_driver2 的 colcon 工作空间（src/ build/ install/）
        ├── build_lightning.sh      lightning 编译脚本
        ├── build_livox_driver.sh   驱动编译脚本
        └── logs/                   各步骤的完整日志
```

运行时用 `ros2 run lightning <程序>`，它跑的是 `install/` 里的那份。`bin/` 下的同名程序带有指向 `build/` 目录的 RUNPATH，如果为了省空间删掉 `build/`（1.3 GB），`bin/` 里的程序会找不到动态库。

apt 装的系统库只能装进 `/usr`，这是 dpkg 的机制决定的，但它们的 .deb 安装包都留在 `thirdparty/deps_build/apt/archives/` 里。

## 遇到的问题和处理

### 1. 装 `libpcl-dev` 报依赖冲突

现象：

```
libusb-1.0-0-dev : Depends: libusb-1.0-0 (= 2:1.0.25-1ubuntu1) but 2:1.0.25-1ubuntu2 is to be installed
libhwloc-dev : Depends: libhwloc15 (= 2.7.0-2) but 2.7.0-2ubuntu1 is to be installed
E: Unable to correct problems, you have held broken packages.
```

原因：系统镜像是带着 `jammy-updates` 源做出来的，有 210 个包装的是 updates 版本；但 `/etc/apt/sources.list` 里后来把 `jammy-updates` 删掉了，只剩注释掉的 `deb-src`。于是所有要求运行库版本严格一致的 `-dev` 包都装不上。

处理：没有改 `/etc/apt`（看起来是有意锁定的）。在项目目录下单独建了一份 apt 配置 `thirdparty/deps_build/apt/apt.conf`，用自己的 `sources.list`（包含 jammy-updates）、自己的索引目录和下载缓存，只在安装本库依赖时用：

```bash
A=~/lightning-lm/thirdparty/deps_build/apt
sudo env APT_CONFIG=$A/apt.conf apt-get update -o APT::Sandbox::User=root
sudo env APT_CONFIG=$A/apt.conf apt-get install -o APT::Sandbox::User=root \
    libpcl-dev libgoogle-glog-dev libgflags-dev ros-humble-pcl-conversions libepoxy-dev
```

影响：新装 306 个包，占 1.2 GB。PCL 会连带装上 VTK9、Qt5、openjdk-11 等。另外有 13 个已装的包被升到了安全更新版本（curl、libfreetype6、libpoppler 等，jammy-security 本来也会推这些）。没有删除任何包，没有动 CUDA 和 NVIDIA 的包。

### 2. ports.ubuntu.com 太慢

实测约 0.4 MB/s，306 MB 要十几分钟。上面那份专用 `sources.list` 改用了 USTC 镜像 `https://mirrors.ustc.edu.cn/ubuntu-ports`，约 20 MB/s。清华镜像返回 403，阿里云约 12 MB/s 也能用。

### 3. apt 报 "Download is performed unsandboxed as root"

apt 默认用 `_apt` 用户下载，这个用户没权限写 `/home/dm/...`。只是警告，不影响安装；加 `-o APT::Sandbox::User=root` 就不报了。代价是 `apt/lists`、`apt/archives` 归 root 所有，要删的话得用 sudo。

### 4. CMake 报 `/usr/lib/libopencv_core.so.4.8.0` 不存在

```
CMake Error at /usr/lib/cmake/opencv4/OpenCVModules.cmake:172 (message):
  The imported target "opencv_core" references the file "/usr/lib/libopencv_core.so.4.8.0"
  but this file does not exist.
```

原因：设备上只装了 NVIDIA 的 `libopencv-dev 4.8.0`（头文件、CMake 文件、`.so` 软链接），没装对应的运行库包 `libopencv`，所以软链接指向的文件不存在。另外系统里还装着 Ubuntu 的 OpenCV 4.5.4 运行库（`ros-humble-cv-bridge`、`python3-opencv` 依赖它），它的开发包早先被 NVIDIA 的 `libopencv-dev` 替换掉了。

处理：补装 NVIDIA 的运行库 `libopencv 4.8.0-1-g6371ee1`，同样用上面那份专用 apt 配置。事先模拟过，不会删除 Ubuntu 的 4.5 运行库：两者 soname 不同、目录也不同（`/usr/lib` 和 `/usr/lib/aarch64-linux-gnu`），可以共存。lightning 只链接 4.8（已用 `ldd` 确认）。

注意：以后如果有程序要同时链接 lightning 和 `cv_bridge`，同一个进程里会出现两个版本的 OpenCV，要避免。

### 5. `-msse` 在 aarch64 上不认

`cmake/packages.cmake:31-32` 在没打开 `BUILD_WITH_MARCH_NATIVE` 时会写死 `-msse … -msse4.2`，aarch64 的 gcc 直接报错：

```
g++: error: unrecognized command-line option '-msse'
```

处理：编译时加 `--cmake-args -DBUILD_WITH_MARCH_NATIVE=ON`。实际得到的指令集是 `-march=armv8.2-a+crypto+fp16+rcpc+dotprod+flagm+pauth`。

注意：这样编出来的程序和 CPU 绑定。Orin 系列（Nano/NX/AGX，都是 A78AE 核）之间可以通用，换别的 ARM 芯片要重新编译。源码里没有 x86 内联汇编或 SSE 指令，只改这个参数就够了。

### 6. 无头设备上也必须编 Pangolin

`find_package(Pangolin REQUIRED)`，而且 UI 代码一定会参与编译，所以 Pangolin 不能省；运行时把 `with_ui` 关掉就不会去开窗口。

处理：用仓库自带的 `thirdparty/Pangolin-0.9.3.zip`，不用联网下载，编成静态库装到 `thirdparty/deps_install`。编译选项和开发机一致：关掉 examples、tools、python、ffmpeg 等。编 lightning 时用 `-DPangolin_DIR=.../deps_install/lib/cmake/Pangolin` 指定位置。

另外：装 PCL 时连带装了 `libopenni2-dev`，Pangolin 会自动启用 OpenNI2，所以程序会多链接一个 `libOpenNI2.so`。不影响使用。

### 7. 编译内存

单个编译 PCL 的源文件最多吃到约 2.4 GB 内存（CMake 写死了 `-g -ggdb`）。设备只有 7.4 GB，用 `-j2` 并发 2，再用 systemd 限制最多 5 GB 内存加 2 GB swap，这样即使超限也只会杀掉编译进程，不会把整台机器卡死。实际 swap 只用了几十 MB。`-j2` 总耗时约 20 分钟。

### 8. SSH 断开会中断编译

设备上没有 tmux 和 screen，用户也没开 linger（`loginctl` 显示 `Linger=no`）。处理：用系统级的 systemd 临时服务跑编译，断开 SSH 也不受影响，同时带上内存上限：

```bash
sudo systemd-run --unit=lightning-build --uid=dm --gid=dm \
    -p MemoryMax=5G -p MemorySwapMax=2G -p Nice=10 \
    --setenv=HOME=/home/dm --setenv=JOBS=2 \
    /home/dm/lightning-lm/thirdparty/deps_build/build_lightning.sh
# 查看进度：tail -f ~/lightning-lm/thirdparty/deps_build/logs/colcon_build.log
```

### 9. `ros2 bag play` 不播 `/livox/lidar`

```
Ignoring a topic '/livox/lidar', reason: package 'livox_ros_driver2' not found
```

原因：`thirdparty/livox_ros_driver` 是被 lightning 用 `add_subdirectory` 编进来的，只给 lightning 自己用，没有注册成 ROS 包，所以 `ros2 bag`、`ros2 topic echo` 这些工具找不到这个消息类型。

处理：现在真正的驱动已经装好（见"Livox SDK 和驱动"一节），回放前在那个 shell 里 `source ~/lightning-lm/thirdparty/deps_build/livox_ws/install/setup.bash` 就行。最初测试时临时编过一个只含消息定义的包，驱动装好后已删除。

### 10. 无头运行要关 UI

`config/default_livox.yaml` 里是 `with_ui: true`，在无头设备上要关掉。为此新建了 `config/default_livox_jetson.yaml`，和原配置只差 `with_ui: false` 这一项。它由 `build_lightning.sh` 在每次编译前从 `default_livox.yaml` 重新生成，不要手改；它没有进仓库，在 Jetson 的 `.git/info/exclude` 里忽略。注意离线建图会把结果存到**当前目录**下的 `./data/new_map`，存之前会先删掉这个目录，所以测试都在 `data/jetson_test/` 下跑，避免覆盖开发机同步过来的 `data/new_map`。

### 11. 运行时看到的现象（不是 Jetson 特有）

- **PGO 大量报 DR 时间戳倒退。** 日志是 `当前DR定位的结果的时间戳应当比上一个时间戳数值大，实际相减得-2.3…`，在线定位出现了 652 次。开发机 x86 上的离线日志 `loc_1.log` 里也有 1461 次（最多倒退 1.0 秒），说明是上游代码本身的行为，和移植无关。被这样拒掉的 IMU 推算帧会被 PGO 丢弃，值得单独排查。
- **在线定位刚启动时 LIO 队列丢了 6 帧**（`激光里程计 exceeds largest size: 1`），都集中在回放开头 20 ms 内，是 bag 起始的突发数据，之后没再出现。
- **bag 播完后报 `长时间未获取到DR数据, IMU存在断流`**，这是数据停了的正常反应。

## Livox SDK 和驱动

### 版本

| 组件 | 版本 | 位置 |
|---|---|---|
| Livox-SDK2 | tag `v1.3.1`（f5d9375） | 源码 `thirdparty/deps_build/src/Livox-SDK2`，装在 `thirdparty/deps_install`（`liblivox_lidar_sdk_shared.so`、静态库和 3 个头文件） |
| livox_ros_driver2 | `13eb05e`（"support Mid-360s"，驱动自报版本 1.2.6） | `thirdparty/deps_build/livox_ws` |

选这两个版本，是为了和开发机 `/opt/DeepMirror` 下在用的组合保持一致（SDK 1.3.1 + 驱动 13eb05e）。上游驱动的 master 比它多两个提交（`2144554`，加了 Mid360L、Avia2 支持，修了"配置项全省略时不发点云"的问题），只用 Mid-360 的话用不到。两个仓库都直接从 GitHub 克隆，设备上访问 GitHub 很快。

SDK 编译一次就过（`make -j4`，无错误，编译输出里有 24 行 warning）。驱动编译 58 秒，无警告。

### 遇到的问题

**a. 驱动的 CMake 写死了 SDK 的位置。** `find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_shared.so /usr/local/lib REQUIRED)` 默认去 `/usr/local` 找。为了不装进系统目录，编译时直接把路径传进去：`-DLIVOX_LIDAR_SDK_LIBRARY=.../deps_install/lib/liblivox_lidar_sdk_shared.so -DLIVOX_LIDAR_SDK_INCLUDE_DIR=.../deps_install/include`。

**b. 运行时找不到 SDK 动态库。** SDK 不在系统库路径里。编译时加 `-DCMAKE_INSTALL_RPATH=.../deps_install/lib`，装好的 `liblivox_ros_driver2.so` 会带上这个 RUNPATH，不用另设 `LD_LIBRARY_PATH`。

**c. 官方 `build.sh` 不适合直接用。** 它会 `rm -rf ../../build ../../install`，把 `package_ROS2.xml` 复制成 `package.xml`，然后用固定参数调 colcon，没法加上面 a、b 的参数。所以写了 `build_livox_driver.sh`，做同样的事（复制 `package_ROS2.xml`、传 `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble`），再加上 SDK 路径和 RPATH，同样用 systemd-run 加内存上限来跑。

**d. 驱动初始化失败后进程关不掉。** 没接雷达时（或 `MID360_config.json` 里的本机 IP 不在任何网卡上）会报：

```
bind failed
Failed to init livox lidar sdk.
[ERROR] [livox_lidar_publisher]: Init lds lidar fail!
```

之后进程不退出，Ctrl-C（SIGINT）和 SIGTERM 都不响应，`ros2 launch` 要等 15 秒才升级成 SIGKILL 杀掉它。这是上游驱动的行为。用 systemd 或 launch 管理时，要么把停止超时设短，要么在启动前先确认网络和配置是对的。

**e. 和 lightning 里自带的消息库同名。** lightning 自带的消息定义也会生成 `liblivox_ros_driver2__rosidl_typesupport_*.so`，和驱动生成的文件名完全一样。在同一个 shell 里先 source 驱动、再 source lightning，`LD_LIBRARY_PATH` 里 lightning 的目录排在前面，从这个 shell 启动的驱动会加载到 lightning 那份。两份都是从同一组 `.msg` 生成的，实测可以互通：用驱动的消息包回放 bag，lightning 收到了 58 帧，定位也初始化成功。稳妥起见，驱动和 lightning 最好在各自的 shell 或 launch 环境里启动：

```bash
# 终端 1：驱动
source /opt/ros/humble/setup.bash
source ~/lightning-lm/thirdparty/deps_build/livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py      # 发布 CustomMsg：/livox/lidar 和 /livox/imu

# 终端 2：lightning
source /opt/ros/humble/setup.bash
source ~/lightning-lm/install/setup.bash
ros2 run lightning run_loc_online --config ~/lightning-lm/config/default_livox_jetson.yaml
```

`msg_MID360_launch.py` 默认 `xfer_format=1`（CustomMsg），话题名和 `default_livox.yaml` 里的 `/livox/lidar`、`/livox/imu` 一致，不用改。

### 接雷达前还要做的

设备目前只有一个网口 `enP8p1s0`（10.198.1.219，办公网），没接雷达，所以还没做真机测试。接雷达时需要：

1. **给连雷达的网口配一个和雷达同网段的地址。** 上游默认配置里雷达是 `192.168.1.12`，本机是 `192.168.1.5`。开发机上的配置用的是雷达 `192.168.123.120`、本机 `192.168.123.165`。Orin Nano 开发板只有一个网口，如果雷达和办公网共用这个口（经交换机），就要在 `enP8p1s0` 上再加一个地址。
2. **改 `livox_ws/src/livox_ros_driver2/config/MID360_config.json`**：`host_net_info` 里的各个 IP 改成本机地址，`lidar_configs[0].ip` 改成雷达地址。然后重跑 `build_livox_driver.sh`（约 1 分钟），因为装好的那份配置是编译时复制进 `install/` 的。

## 复现步骤（换一台同镜像的 Orin）

```bash
# 开发机：同步源码（不带编译产物）
rsync -a --exclude=/build/ --exclude=/install/ --exclude=/log/ --exclude=/bin/ \
      --exclude=/thirdparty/deps_build/ --exclude=/thirdparty/deps_install/ \
      ./ dm@<ip>:~/lightning-lm/

# Jetson：建专用 apt 配置（见问题 1、2）
A=~/lightning-lm/thirdparty/deps_build/apt
mkdir -p $A/lists/partial $A/archives/partial $A/cache
cat > $A/sources.list <<EOF
deb https://mirrors.ustc.edu.cn/ubuntu-ports/ jammy main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu-ports/ jammy-updates main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu-ports/ jammy-security main restricted universe multiverse
EOF
cat > $A/apt.conf <<EOF
Dir::Etc::SourceList "$A/sources.list";
Dir::State::Lists "$A/lists";
Dir::Cache "$A/cache";
Dir::Cache::archives "$A/archives";
EOF
# /etc/apt/sources.list.d/ 里的 NVIDIA 源和 ROS 源仍会照常读取

# Jetson：安装依赖
sudo env APT_CONFIG=$A/apt.conf apt-get update -o APT::Sandbox::User=root
sudo env APT_CONFIG=$A/apt.conf apt-get install -y -o APT::Sandbox::User=root \
    libpcl-dev libgoogle-glog-dev libgflags-dev ros-humble-pcl-conversions libepoxy-dev libopencv

# Jetson：编 Pangolin
cd ~/lightning-lm/thirdparty/deps_build && mkdir -p src pangolin_build && touch COLCON_IGNORE
(cd src && unzip -q ../../Pangolin-0.9.3.zip)
cd pangolin_build && cmake ../src/Pangolin-0.9.3 -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/lightning-lm/thirdparty/deps_install -DBUILD_SHARED_LIBS=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_TESTS=OFF -DBUILD_PANGOLIN_PYTHON=OFF \
  -DBUILD_PANGOLIN_FFMPEG=OFF -DBUILD_PANGOLIN_LIBTIFF=OFF -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
  -DBUILD_PANGOLIN_LIBRAW=OFF -DBUILD_PANGOLIN_ZSTD=OFF -DBUILD_PANGOLIN_REALSENSE=OFF \
  -DBUILD_PANGOLIN_OPENNI=OFF -DBUILD_PANGOLIN_LIBUVC=OFF -DBUILD_PANGOLIN_DEPTHSENSE=OFF \
  -DBUILD_PANGOLIN_PLEORA=OFF -DBUILD_PANGOLIN_TELICAM=OFF -DBUILD_PANGOLIN_LIBDC1394=OFF
make -j4 && make install

# Jetson：编 lightning（见问题 8）
sudo systemd-run --unit=lightning-build --uid=dm --gid=dm -p MemoryMax=5G -p MemorySwapMax=2G \
    --setenv=HOME=/home/dm --setenv=JOBS=2 /home/dm/lightning-lm/thirdparty/deps_build/build_lightning.sh
```

`build_lightning.sh` 的核心就是这一条：

```bash
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 colcon build --parallel-workers 1 --cmake-args -DBUILD_WITH_MARCH_NATIVE=ON \
    -DPangolin_DIR=$HOME/lightning-lm/thirdparty/deps_install/lib/cmake/Pangolin
```

Livox SDK 和驱动：

```bash
P=~/lightning-lm/thirdparty
git clone --depth 1 --branch v1.3.1 https://github.com/Livox-SDK/Livox-SDK2.git $P/deps_build/src/Livox-SDK2
mkdir -p $P/deps_build/livox_sdk2_build && cd $P/deps_build/livox_sdk2_build
cmake ../src/Livox-SDK2 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$P/deps_install
make -j4 && make install

mkdir -p $P/deps_build/livox_ws/src && cd $P/deps_build/livox_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git && git -C livox_ros_driver2 checkout 13eb05e
# 先按"接雷达前还要做的"改好 config/MID360_config.json，再编译
sudo systemd-run --unit=livox-driver-build --uid=dm --gid=dm -p MemoryMax=5G -p MemorySwapMax=2G \
    --setenv=HOME=/home/dm --setenv=JOBS=2 /home/dm/lightning-lm/thirdparty/deps_build/build_livox_driver.sh
```

`build_livox_driver.sh` 的核心：

```bash
cp -f package_ROS2.xml package.xml          # 在 livox_ws/src/livox_ros_driver2 下
colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DBUILD_TESTING=OFF \
  -DLIVOX_LIDAR_SDK_LIBRARY=$P/deps_install/lib/liblivox_lidar_sdk_shared.so \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR=$P/deps_install/include \
  -DCMAKE_INSTALL_RPATH=$P/deps_install/lib  # 在 livox_ws 下
```

## 还没做 / 建议

- **还没接雷达做真机测试。** 网络和配置要改的地方见"接雷达前还要做的"。
- **设备虽然无头，但 GDM 在跑，默认启动目标是 `graphical.target`。** 切到 `multi-user.target` 能省下桌面占的内存。本次没改。
- **限制线程。** lightning 的并行用 TBB，默认会占满 6 个核，NDT 又写死了 4 线程。和感知、规划一起跑时，建议用 `systemd-run -p AllowedCPUs=` 或 `taskset` 限制可用的核。
- **关掉逐帧日志。** 在线程序每帧都打好几行 INFO 日志。正式运行时可以设 `GLOG_minloglevel=1` 关掉，做评估（`scripts/eval_loc_log.py`）时再保留。
- **可以删索引省空间。** `thirdparty/deps_build/apt/lists`（341 MB）是 apt 索引，装完可以 `sudo rm -rf` 掉；`archives/` 里的 .deb 建议留着，以后可以离线重装。
