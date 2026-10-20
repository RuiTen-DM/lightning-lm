# 订阅 IMU 若干秒，每个 1 秒窗口里加计模长的标准差都小于阈值（默认 4 mg）才算静止；退出码 0 = 静止
import sys, time, numpy as np, rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
th = float(sys.argv[2]) if len(sys.argv) > 2 else 0.004  # g
rclpy.init(); n = rclpy.create_node('still_check'); s = []
n.create_subscription(Imu, '/livox/imu', lambda m: s.append((time.time(), np.linalg.norm(
    [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]))), QoSProfile(depth=500))
t0 = time.time()
while time.time() - t0 < secs + 1.5: rclpy.spin_once(n, timeout_sec=0.1)
a = np.array(s)
if len(a) < 100 * secs: print('too few imu'); sys.exit(2)
a = a[a[:, 0] > a[-1, 0] - secs]
stds = [a[(a[:, 0] >= t) & (a[:, 0] < t + 1), 1].std() for t in np.arange(a[0, 0], a[-1, 0] - 1, 1.0)]
print('max 1 s std %.2f mg' % (max(stds) * 1000)); sys.exit(0 if max(stds) < th else 1)
