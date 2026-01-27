# Package Name

## 简介

简要说明这个包的功能和用途。

## 功能特性
- 功能1: 描述
- 功能2: 描述
- 功能3: 描述

## 环境要求
- **ROS2版本**: Humble
- **操作系统**: Ubuntu 22.04 LTS
- **编程语言**: C++17, Python 3.10
- **外部依赖**:
  - navigation2
  - moveit2
  - mavlink
  - snap7
  - other...

## 依赖项

### ROS2包依赖
```xml
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sealien_rov_msg</depend>
  <depend>geometry_msgs</depend>
```

### 系统依赖
```bash
sudo apt install libopencv-dev libeigen3-dev
```

### Python依赖(如有)
```bash
pip3 install numpy scipy
```

## 安装

### 从源码构建
```bash
cd ~/ros2_ws/src
git clone https://github.com/your-org/package_name.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select package_name
```

## 使用方法

### 基本启动
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch package_name default.launch.py
```

### 启动文件说明

#### 1. default.launch.py
默认配置启动,适用于大多数场景。

**参数:**
- `param1` (string, default: "value"): 参数说明
- `param2` (int, default: 10): 参数说明

**示例:**
```bash
ros2 launch package_name default.launch.py param1:=custom_value
```


#### 2. <packge_name_sim>.launch.py
仿真环境启动, 使用仿真传感器数据或者虚拟数据文件

**示例:**
```bash
ros2 launch package_name <packge_name_sim>.launch.py use_rviz:=true
```

#### 3. <packge_name>.launch.py
真实硬件启动, 连接物理传感器。

**前置条件:**
- 硬件已正确连接
- 接口权限已配置

**示例:**
```bash
ros2 launch package_name <packge_name>.launch.py device:=/dev/ttyUSB0
```

#### 4. <packge_name_test>.launch.py
功能自测。

**前置条件:**
- 硬件已正确连接
- 传感器权限已配置
- ...

**示例:**
```bash
ros2 launch package_name <packge_name_test>.launch.py
```

### 配置文件说明

#### default_params.yaml
```yaml
node_name:
  ros__parameters:
    param1: value1      # 参数1说明
    param2: 10          # 参数2说明,单位:米
    param3: true        # 参数3说明
```

## 节点说明

### node_main
主节点, 执行核心功能。

**订阅的话题:**
| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/input_topic` | `sensor_msgs/Sonar` | 声呐数据 |
| `/camera/image` | `sensor_msgs/Image` | 相机图像 |

**发布的话题:**
| 话题名称 | 消息类型 | 描述 | 频率 |
|---------|---------|------|------|
| `/output_topic` | `geometry_msgs/Twist` | 速度指令 | 10Hz |
| `/status` | `std_msgs/String` | 节点状态 | 1Hz |

**提供的服务:**
| 服务名称 | 服务类型 | 描述 |
|---------|---------|------|
| `/reset` | `std_srvs/Trigger` | 重置节点状态 |

**使用的动作:**
| 动作名称 | 动作类型 | 描述 |
|---------|---------|------|
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | 导航到目标 |

**参数:**
| 参数名称 | 类型 | 默认值 | 描述 |
|---------|------|--------|------|
| `max_speed` | double | 1.0 | 最大速度(m/s) |
| `update_rate` | int | 10 | 更新频率(Hz) |

## 已知问题

1. **问题1**: 描述
   - **临时解决方案**: 描述
   - **跟踪**: 在改了在改了，用的时候你躲远点

## 稳定功能说明（标注好Tag, 便于后续使用）

- [ ] 功能A (Tag: v1.1.0)
- [ ] 功能B (Tag: v1.2.0)
- [x] 功能C (Tag: v1.0.0)
- [ ] 功能A&B&C (Tag: v1.0.0)

## 版本历史

查看 [CHANGELOG.md](CHANGELOG.md)

## 相关资源

- [ROS2文档](https://docs.ros.org)
- [参考论文](url)