# 持续运行的里程计 run_lio_online

记录时间：2026-10-15　分支：`feat/lio-online`

只跑激光惯性里程计（LIO），不建图，也不用地图。用于吊轨机器人这类在固定轨道上长期往复运动的场景。要不要同时用保存的地图做实时定位来修正，还没定。

## 用法

```bash
ros2 run lightning run_lio_online --config ./config/default_livox.yaml   # Jetson 上用 default_livox_jetson.yaml
```

输出 `odom -> base_link` 的位姿，频率和 IMU 相同（200 Hz）：

| 输出 | 说明 |
|---|---|
| `lightning/odom`（`nav_msgs/Odometry`） | 位姿在 `odom` 系下，twist 在 `base_link` 系下。`base_link` 就是 IMU 系 |
| TF `odom -> base_link` | 可以用 `pub_tf` 关掉 |

两次激光更新之间，按匀速和陀螺做外推。超过 `max_extrapolation` 秒（默认 0.5 秒）没有激光更新就停止发布，避免外推越跑越偏。

开了 `fasterlio/gravity_align` 时，`odom` 系的 Z 轴朝上。如果以后要加地图定位来修正，可以按 ROS 的惯例由定位发布 `map -> odom`。注意 `run_loc_online` 现在发布的是 `map -> base_link`，两个程序同时跑时 TF 会冲突，到时要改。

## 配置

`config/default_livox.yaml`：

```yaml
fasterlio:
  ivox_grid_resolution: 0.55
  ivox_capacity: 100000      # 局部地图最多保留的格子数，不写时为 100 万

lio_online:
  odom_topic: "lightning/odom"
  odom_frame: "odom"
  base_frame: "base_link"
  pub_tf: true
  max_extrapolation: 0.5
  status_interval: 10.0      # 每隔多少秒打印一次运行状态
```

## 为长期运行做的处理

- **不保留历史关键帧。** LIO 用非建图模式运行，内存里只留最近一帧。
- **局部地图有上限。** 局部地图由 iVox 格子组成，格子数超过 `ivox_capacity` 就淘汰最久没加过点的格子。每个格子最多存 9 个点，新点进来时挤掉最旧的点。所以搜索开销和内存都有上限。
  - 这条轨道来回跑 10 轮后约 7 万格。上限 20 万时一直没有触发淘汰；现在设为 10 万，如果下方垛堆变化让格数涨到上限，就会开始淘汰。
  - 上限低于轨道所需的格子数时会持续淘汰。实测这种情况下不会出错，但高度会慢慢漂：上限设 3 万时，回到起点处的高度从 -0.01 m 漂到了 -0.48 m。
  - 注意淘汰顺序是按"最后一次往格子里加点"来算的，不是按"最后一次被用来匹配"。已经填满、不再加点的静态结构，反而会先被淘汰。
- **输入缓存有上限。** 雷达停了而 IMU 还在时，IMU 会越攒越多（每天约 1.7 GB）；IMU 停了时，点云会越攒越多。现在两者都会被限制住：
  - IMU 只保留最近 2 秒。
  - 待处理的点云最多 10 帧。
- **单线程处理。** 所有回调都在一个线程里执行。每收到一帧点云，就把 IMU 已经覆盖到的缓存点云都处理掉，延迟不会越积越多。
- **日志。** glog 默认只记 WARNING 及以上，需要逐帧日志时加 `--minloglevel=0`。运行状态由 ROS 日志定期打印，格式如下：

```
frames 33194/33196 scans, dropped 0, imu ..., odom ... | lio 20.71 ms avg, 25.88 ms max | local map 70302 grids | rss 94.9 MB | pos ...
```

## 没有雷达时的测试：循环放包

`scripts/bag_loop_player.py` 把多个包首尾相接地循环播放。它会改写 `header.stamp`（Livox CustomMsg 的 `timebase` 也一起改），保证时间一直往前走。它直接改 CDR 字节，播放 20 万点/秒也不怎么占 CPU。

```bash
# 终端 1：里程计
source /opt/ros/humble/setup.bash && source ~/lightning-lm/install/setup.bash
ros2 run lightning run_lio_online --config ~/lightning-lm/config/default_livox_jetson.yaml
# 终端 2：放包。要 source 驱动的消息包，不要 source lightning（同名消息库，见 jetson_deploy.md 问题 e）
source /opt/ros/humble/setup.bash && source ~/lightning-lm/thirdparty/deps_build/livox_ws/install/setup.bash
python3 ~/lightning-lm/scripts/bag_loop_player.py \
    ~/lightning-lm/data/bags/rosbag2_2026_09_29-10_42_14_custom \
    ~/lightning-lm/data/bags/rosbag2_2026_09_29-10_45_14_custom --loops 0 --rate 1
```

10_42 和 10_45 正好一去一回，首尾基本能接上。这是把两张地图互相配准后确认的：

- 10_45 的起点和 10_42 的终点相差 2 cm。
- 10_45 的终点离 10_42 的起点 0.27 m，朝向差约 1°。

两段的接缝处会有 0.25–0.4 秒的 IMU 空档，日志里会因此出现 `get abnormal dt`。10_45 包本身中间也有一段 0.6 秒的 IMU 断流。这些都是数据本身的问题。

## 测试结果

### x86（i7-11800H），4 倍速，3 轮

| ivox_capacity | 每帧 LIO | 局部地图 | 常驻内存 | 每轮回到远端 / 起点的位置 |
|---|---|---|---|---|
| 100 万（不限） | 1.8 → 2.3 ms | 4.6 万 → 6.5 万格 | 90 → 120 MB | 每轮都在 (-21.85, 178.3, 1.56) / (0.0, 0.3, 0.0)，差几厘米 |
| 3 万 | 约 1.9 ms | 一直 3 万（持续淘汰） | 约 107 MB，不涨 | 远端差约 1 m；起点处高度 -0.38 → -0.46 → -0.48 m |

### Jetson Orin Nano（25W），实时速度，10 轮（57 分钟，约 3.6 km）

- **运行稳定。** 33196 帧点云处理了 33194 帧，0 帧丢弃。odom 发了 68 万条，时间戳没有倒退。最大间隔 0.61 秒，就是包里那段 IMU 断流。
- **不累积漂移。** 每轮回到远端都是 (-22.07, 178.33, 1.48)，回到起点都是 (-0.04, 0.29, -0.01)，前后差 1–2 cm。原因是局部地图一直完整保留着第一轮建的整条轨道。
  - 注意：这是同一段数据反复播放，场景完全一样。真机运行时噪声和下方堆垛都会变，这里只能说明程序稳定、资源有上限，不能代表实际漂移。
- **资源随局部地图变满而上升，后期趋平：**

| 时间 | 局部地图 | 常驻内存 | 每帧 LIO（平均/最大） | CPU（占单核） |
|---|---|---|---|---|
| 第 1 轮 | 4.3 万格 | 68 MB | 13 / 24 ms | 54–71% |
| 第 3 轮 | 5.9 万格 | 77 MB | 17 / 22 ms | 85–88% |
| 第 10 轮 | 7.0 万格 | 97 MB | 20 / 27 ms | 100–107% |

- **核数影响延迟，不影响 CPU 总量。** 这是在第 1 轮实时跑的 10_42 包。整台设备有 6 个核：

| 给里程计的核数 | CPU | 每帧 LIO |
|---|---|---|
| 4 | 54% | 16 ms |
| 2 | 50% | 25 ms |
| 1 | 47% | 45 ms |

另外，测试用的 Python 放包脚本约占 6% CPU，odom 记录脚本约占 26%。

## 结果记录（长期运行后取回）

`lio_online/record_dir` 不为空时，程序把结果精简地记成 csv。时间都用系统时间：

| 文件 | 内容 | 大小 |
|---|---|---|
| `status.csv` | 每个状态间隔一行：帧数、丢帧、每帧耗时、局部地图格数、内存、CPU、位置。每次启动先写一行 `# start`，可以看出有没有重启 | 间隔 60 秒时约 150 KB/天 |
| `stops.csv` | 机器人每次停稳记一行：至少走过 1 m 后静止 3 秒，记录位姿和这一段的路程。只用来标出停车位置；现场没有固定停靠点，不拿它算漂移 | 很小 |
| `still.csv` | 静止时的漂移。用 IMU 判断真正静止：1 秒内加计模长的波动小于 4 mg，且陀螺小于 0.05 rad/s。这台机器人沿轨道平移时几乎不转，陀螺分不出动静，静止时加计约 1–2 mg，电机一转就到 6 mg 以上。记录静止期间里程计位置变了多少；静止至少 30 秒才记，长时间静止时每 10 分钟记一行累计值 | 很小 |
| `traj_日期.csv` | 轨迹：移动时每秒一行，静止时每分钟一行。按天分文件，超过 `record_keep_days`（默认 30）天的删掉 | 一直在动时约 4 MB/天 |

另外可以定期把局部地图存下来。`map_save_interval` 大于 0 时生效，退出时也会存一次：

- `local_map.pcd`：LIO 的局部地图，即 iVox 里所有的点，存 xyz 和强度，压缩格式。
- `local_map_pose.txt`：存图时的位姿、帧数、格数和点数。
- 先在处理线程里拷出点云，再由后台线程写临时文件、改名，不阻塞 LIO。
- 每次启动时，把上次的地图改名为 `local_map_prev.pcd` 保留。上次的地图在另一个坐标系里，因为每次启动都从当时的位置重新开始。
- 实测这条轨道来回 3 轮后约 6 万格、24 万点，文件 3.4 MB，x86 上写一次 14–30 ms。每格最多 9 个点，所以 10 万格上限下文件最大约 13 MB。

汇总和可视化：

```bash
python3 scripts/lio_record_summary.py <记录目录>   # 文字汇总
python3 scripts/lio_record_plot.py <记录目录>      # 生成 <记录目录>/lio_record.html，浏览器打开
```

页面是单个文件，不联网也能看，只依赖 numpy。按每次运行分开显示：

- 俯视图：横轴沿轨道、纵轴横向，等比例；画出局部地图、轨迹和停车位置。
- 沿轨道位置、高度随时间的变化。
- 静止时的漂移：IMU 判断静止期间里程计的位移。
- 来回一致性：轨道是固定的直线，每一趟经过同一沿轨道位置（2 m 一格）时，横向偏移和高度应该相同。这里取各趟相对最早经过那里那一趟的偏差的中位数，随时间变大就是漂移；航向漂移表现为离起点越远、横向偏得越多。沿轨道方向本身的漂移从这里看不出来。
- 最早和最近一趟的横向、高度剖面。
- CPU、内存、每帧耗时、格数、丢帧随时间的变化。

可以滚轮缩放、拖动平移、悬停看数值。地址后加 `#序号` 可以直接打开某一次运行。

内容包括：启动次数和本次运行时长、最新状态、运行以来的最大值，以及每次运行的静止漂移和来回一致性。每次重启后坐标系从当时的位置重新开始，所以按每次运行分开统计。

## 部署到没有 sudo、不方便编译的设备

现场板子上不能装包（没有 sudo），负载也高。所以在同型号的开发板上编译，只拷运行需要的文件过去：

- 程序本体约 8 MB（已去掉调试信息）：`run_lio_online`、`liblightning.libs.so`、`libmiao.*`、`liblightning__rosidl_*`、`liblivox_ros_driver2__rosidl_*`。
- 外加板子上缺的 NVIDIA OpenCV 4.8 运行库（core/highgui/imgcodecs/imgproc）。
- 合计 18 MB，压缩后 6 MB。

拷过去之前，先在开发板上用 `ldd` 列出依赖，再到目标板上逐个确认是否存在。目录结构：

```
~/lightning_lio/
├── bin/run_lio_online
├── lib/                 上面那些库，run.sh 把它加进 LD_LIBRARY_PATH
├── config/lio_field.yaml
├── run.sh               source /opt/ros/humble，glog 只写 stderr（进 journald），ROS 日志放 ros_log/
├── records/             结果记录
└── VERSION              编译所用的提交
```

启动脚本和服务文件在 `scripts/lio_deploy/`（`run.sh`、`lightning-lio.service`）。用 systemd 用户服务 `~/.config/systemd/user/lightning-lio.service` 运行：

- `Restart=always`，`Nice=10`，`MemoryMax=1G`，`Environment=ROS_DOMAIN_ID=0`（和驱动一致）。
- 用户服务要等用户登录后才启动。板子开了 GDM 自动登录，所以重启后会自动起来。
- 如果设备没有自动登录，需要 `sudo loginctl enable-linger <用户>`。

```bash
systemctl --user status lightning-lio            # 状态
journalctl --user-unit=lightning-lio -n 50       # 日志
systemctl --user restart lightning-lio           # 重启
```

现场机器人上 TF 由别的程序管，所以 `lio_field.yaml` 里把 `pub_tf` 关了，只发 `lightning/odom`。

**更新版本时等机器人停稳再重启。** 重启会让里程计原点重置。另外 LIO 启动时用最初 0.1 秒的加计来估计重力方向，机器人在动时启动会带来几度的倾角误差。`scripts/lio_deploy/deploy_when_still.sh` 的做法：

- 先把新版本放到板子的 `/tmp/lio_deploy`；
- 用 `still_check.py` 等到 IMU 平稳：3 秒内每个 1 秒窗口的加计波动都小于 10 mg；
- 再停服务、替换 `bin/`、`lib/` 和配置、启动；
- 旧版本备份在 `~/lightning_lio_backup/<版本>`。

现场是吊轨机器人，停下后会晃：1 秒内的加计波动常在 4–10 mg，要停很久才降到 1–2 mg。所以记录静止漂移用的 4 mg 只会在较长的停放时触发，短暂的停车不算。

**开机顺序的坑。** 2026-10-17 现场板子重启后，里程计按时自动启动了，但之后两天多一帧数据都没收到（`frames 0, imu 0`）。原因：

- 用户服务在 01:17:29 启动，这时网卡还没配好地址，lidar 网口 01:17:32 才就绪。
- Fast DDS 的参与者在网卡配好地址之前创建，之后就一直发现不了网络起来后才出现的雷达驱动。
- 换个新进程就正常了。

所以 `run.sh` 会先用 `ros2 topic echo --once --no-daemon` 确认 IMU 真的有数据再启动。每次检查都是新的参与者，网络起来之后就能通过，正常情况下大约 3 秒。

取回结果：

```bash
rsync -az -e 'ssh -p <端口>' <用户>@<板子>:lightning_lio/records/ ./lio_records/   # 加 --exclude 'traj_*' 只取状态、停车和静止记录
ssh -p <端口> <用户>@<板子> python3 lightning_lio/lio_record_summary.py lightning_lio/records   # 只看汇总
```

## 待定 / 后续

- **是否用地图实时修正。** 如果要做，修正结果用 `map -> odom` 发布。
- **长时间断流后的恢复。** 雷达或 IMU 断流较久后恢复时，LIO 不会重新初始化，只会跳过这段预测（日志里是 `get abnormal dt`）。这时如果机器人已经移动了，有可能跟丢。还没有处理。
- **降低 CPU。** 可以调这些参数，都会影响精度和鲁棒性，需要用真机数据评估：
  - 把 `ivox_nearby_type` 从 18 降到 6，搜索的格子少约三分之二。
  - 加大 `point_filter_num` 或 `filter_size_scan`，每帧点数变少。
  - 减少 `max_iteration`。
- **淘汰策略。** 按"加点时间"淘汰，对"下方堆垛变化导致的过时点"没有针对性。真机长期运行时要观察局部地图格子数，是否会涨到上限。
