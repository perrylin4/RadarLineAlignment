# radar_line_tracker

基于海康威视 MVS 相机 + OpenCV 的激光线检测与机器人对线控制 ROS2 包。

- 运行环境：
  - Ubuntu 24.04
  - ROS 2 Jazzy
  - CMake ≥ 3.22（推荐使用系统自带版本）
  - GCC / G++ ≥ 11

- 功能概要：
  - 从海康威视 MVS 相机实时采集图像；
  - 使用 OpenCV 提取激光线、拟合直线；
  - 通过双 PID 控制机器人姿态：
    - 让激光线在图像中竖直且居中；
    - 横移 + 旋转对齐，误差稳定后缓慢前进；
  - 输出 `geometry_msgs/msg/Twist` 到 `cmd_vel` 话题。

## 1. 依赖包与软件

### 1.1 ROS2 相关

必须已经安装 ROS 2 Jazzy，并且可以正常使用 `colcon` 构建。

主要依赖的 ROS2 包（通过 `find_package` 和 `ament_target_dependencies` 使用）：

- `rclcpp`
- `geometry_msgs`
- `ament_cmake`

ROS2 Jazzy 正常安装后，这些包会自动包含，无需单独安装。

### 1.2 OpenCV

本包使用 OpenCV 进行图像处理，CMake 中通过：

```cmake
find_package(OpenCV REQUIRED)
```

在 Ubuntu 24.04 上，可以直接安装系统自带的 OpenCV 开发包：

```bash
sudo apt update
sudo apt install libopencv-dev
```

### 1.3 海康威视 MVS SDK

用于访问工业相机，需要安装海康威视（Hikrobot） MVS SDK，并确保共享库可被找到。

- 官方下载页面（选择 Linux x86_64 版本）：
  - https://www.hikrobotics.com/cn/machinevision/service/download (中文)
  - https://www.hikrobotics.com/en/machinevision/service/download (英文)

下载并安装后，假设安装路径为：

```bash
/opt/MVS
```

本包的 `CMakeLists.txt` 默认使用该路径：

```cmake
set(MVS_ROOT "/opt/MVS")
include_directories(${MVS_ROOT}/include)
link_directories(${MVS_ROOT}/lib/64)
```

> 如安装在其他目录，请修改包内 `CMakeLists.txt` 中的 `MVS_ROOT` 路径。

#### 1.3.1 运行时库路径配置

确保运行时能找到 `libMvCameraControl.so`，常用方法有两种：

1. **临时设置 `LD_LIBRARY_PATH`**（每个终端会话）：

   ```bash
   export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
   ```

2. **通过 `ldconfig` 全局配置**（推荐）：

   ```bash
   echo "/opt/MVS/lib/64" | sudo tee /etc/ld.so.conf.d/mvs.conf
   sudo ldconfig
   ```

配置完成后，可以用以下命令检查：

```bash
ldconfig -p | grep MvCameraControl
```

## 2. 工程结构

在工作空间中的典型布局：

```bash
ros2_ws/
  src/
    radar_line_tracker/
      CMakeLists.txt
      package.xml
      src/
        line_tracker_node.cpp
      README.md
```

## 3. 在机器人上安装与构建

### 3.1 准备 ROS2 工作空间

在机器人上创建新的工作空间（如尚未创建）：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/jazzy/setup.zsh
```

将此包拷贝到机器人（只需要 `radar_line_tracker` 目录即可）：

```bash
# 在开发机打包
cd /home/you/ros2_ws/src
tar czf radar_line_tracker.tgz radar_line_tracker

# 传到机器人后，在机器人上解压
cd ~/ros2_ws/src
tar xzf ~/radar_line_tracker.tgz
```

### 3.2 安装依赖

在机器人上安装 OpenCV 开发包（如果还未安装）：

```bash
sudo apt update
sudo apt install libopencv-dev
```

安装并配置海康威视 MVS SDK（参考上面的 1.3 部分）。

### 3.3 构建本包

在机器人上：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select radar_line_tracker
```

构建完成后，每次新开终端运行前需要：

```bash
cd ~/ros2_ws
source install/setup.zsh
```

## 4. 运行节点

确认相机已接入并被 MVS SDK 识别，然后在机器人上运行：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

ros2 run radar_line_tracker line_tracker_node
```

默认行为：

- 自动枚举相机，选择索引为 0 的相机；
- 从相机抓取图像，进行阈值 + 形态学 + 直线拟合；
- 通过 PID 控制机器人，使激光线在图像中竖直且居中；
- 优先使用横向移动（`linear.y`）和角速度（`angular.z`）对齐；
- 当角度和中心误差足够小且稳定一段时间后，缓慢向前移动（`linear.x`）。

> 机器人底盘需要订阅 `cmd_vel` 并支持 `linear.y`（全向/麦轮底盘）。若是纯差速底盘，请根据需要调整控制逻辑。

## 5. 可调参数

所有主要参数集中在 `src/line_tracker_node.cpp` 顶部：

- 图像处理：`kThresh`, `kMorphKernelSize`, `kRefineIterations`, `kInitialBandWidth` 等；
- 误差滤波：`kErrorFilterAlpha`；
- PID：`kAngleKp/Ki/Kd`, `kCenterKp/Ki/Kd` 及其输出上下限；
- 运动控制：`kMaxLateralSpeed`, `kCenterMaxRatio`；
- 稳定判定与前进：`kAngleStableThresh`, `kCenterStableRatio`, `kRequiredStableTime`, `kForwardSpeedWhenStable`；
- 调试显示缩放：`kDisplayScale`；
- 调试开关宏：`ENABLE_DEBUG_VIEW`（1 显示调试窗口，0 只控制不显示）。

修改这些常量后，重新运行：

```bash
cd ~/ros2_ws
colcon build --packages-select radar_line_tracker
source install/setup.zsh
```

## 6. 常见问题

1. **编译时报找不到 `MvCameraControl.h` 或 `libMvCameraControl.so`**
   - 检查 `MVS_ROOT` 路径是否正确；
   - 确认已安装 MVS SDK，并且 `include/`、`lib/64/` 目录存在；
   - 使用 `ldconfig` 或 `LD_LIBRARY_PATH` 配置运行时库搜索路径。

2. **运行时报 `libMvCameraControl.so: cannot open shared object file`**
   - 说明运行时未找到 MVS 的共享库；
   - 按照 1.3.1 中的方法配置库路径后重试。

3. **VS Code 中 `#include "rclcpp/rclcpp.hpp"` 标红**
   - 通常是 IDE 没有使用 ROS2 的编译数据库；
   - 在工作空间运行：
     ```bash
     colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
     ```
   - 然后在 VS Code C/C++ 插件设置中，将 `compile_commands.json` 指向 `build/radar_line_tracker/compile_commands.json`。

---

如需根据具体机器人（底盘类型、坐标系）调整控制逻辑，可在 `line_tracker_node.cpp` 中修改：

- `cmd.linear.x`, `cmd.linear.y`, `cmd.angular.z` 的组合方式；
- PID 和阈值参数，以获得更平滑或更灵敏的行为。 
