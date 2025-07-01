# Webots\_Steering\_Wheel

[](https://www.google.com/search?q=LICENSE)

这是一个用于舵轮底盘(Steering Wheel)的Webots仿真项目。项目包含三轮（3WIS）和四轮（4WIS）两种全向底盘的运动学模型和控制器实现。

舵轮运动学分析详情请见我的博客文章：[舵轮运动学分析](https://www.zxytql.top/Algorithm/Steering_Wheel)

## 🌟 项目亮点

  * **两种底盘结构**:
    
      * **三轮全向底盘 (3WIS)**: 实现了三轮结构的运动学解算与控制
        
      * **四轮全向底盘 (4WIS)**: 实现了四轮结构的运动学解算与控制
        
  * **控制算法**:
      * **PID 控制**: 应用于位置闭环控制
        
      * **模糊PID (Fuzzy-PID)**: 用于实现更精确和鲁棒的导航与位置控制
        
      * **轨迹跟踪**: 在四轮底盘控制器中，实现了基于预设路径点的轨迹跟踪功能
        
  * **模块化代码**: 控制器代码结构清晰，分为底盘、传感器、PID导航等多个模块，方便理解和复用。

## 📂 项目结构

```
Webots_Steering_Wheel/
├── 3WIS/                           # 三轮全向底盘
│   ├── controllers/
│   │   └── my_controller/          # 3WIS 控制器
│   │       ├── chassis.c
│   │       ├── chassis.h
│   │       ├── sensor.c
│   │       ├── sensor.h
│   │       └── my_controller.c     # 主程序
│   └── worlds/
│       └── 3WIS_robot.wbt          # Webots 世界文件
│
├── 4WIS/                           # 四轮全向底盘
│   ├── controllers/
│   │   └── PID_Track/              # 4WIS 控制器 (含PID轨迹跟踪)
│   │       ├── bsp_gps.c
│   │       ├── bsp_gps.h
│   │       ├── bsp_imu.c
│   │       ├── bsp_imu.h
│   │       ├── bsp_spvs.c
│   │       ├── bsp_spvs.h
│   │       ├── chassis.c
│   │       ├── chassis.h
│   │       ├── fuzzy_pid.c
│   │       ├── fuzzy_pid.h
│   │       ├── PID_Nav.c
│   │       ├── PID_Nav.h
│   │       ├── path.c
│   │       ├── path.h
│   │       └── main.c              # 主程序
│   └── worlds/
│       └── 1.wbt                   # Webots 世界文件
│
└── README.md                       # 本文档
```

## ⚙️ 技术细节

### 三轮底盘 (3WIS)

  * **运动学模型**: 控制器 `chassis.c` 和 `chassis.h` 中包含了三轮底盘的运动学正解与逆解算法
    
  * **控制方式**: 支持在机器人坐标系或全局坐标系下进行速度控制。控制器通过分解目标速度 `(vx, vy, wz)` 到每个车轮的转向角度和驱动速度来实现全向移动
    
  * **传感器**: 使用了IMU和GPS来获取机器人的姿态和位置信息

### 四轮底盘 (4WIS)

  * **运动学模型**: 与三轮类似，在 `chassis.c` 和 `chassis.h` 中实现了四轮的运动学模型
    
  * **轨迹跟踪**: `PID_Nav.c` 文件中定义了轨迹跟踪的逻辑
    
      * 通过 `path.c` 中预设的路径点进行导航
      * 使用模糊PID控制器 (`fuzzy_pid.c`) 计算X、Y方向的速度，以精确跟踪目标点
      * 当机器人接近最后一个路径点时，会切换到精度更高的点对点跟踪模式
        
  * **传感器**: 同样集成了IMU、GPS和Supervisor来获取速度、位置和姿态数据

## 🚀 如何开始

### 环境依赖

  * Webots: `R2023a` 或更高版本
  * C/C++ 编译器 (例如 GCC, MinGW, or MSVC)

### 运行仿真

1.  克隆或下载本仓库到本地
2.  使用 Webots 打开 `3WIS/worlds/3wis.wbt` 或 `4WIS/worlds/4wis_pid.wbt` 文件
3.  Webots 会自动编译对应的控制器代码
4.  点击仿真界面上的 "运行" 按钮来开始仿真

### 自定义控制

  * **三轮底盘**:

      * 在 `3WIS/controllers/my_controller/my_controller.c` 的 `main` 函数循环中，修改 `Helm_Chassis_Ctrl` 函数的参数 `(vx, vy, wz)` 来改变机器人的运动
    
    ```c
    // 示例: vx=0, vy=0, wz=45 (°/s)
    Helm_Chassis_Ctrl(0, 0, 45, &helm_chassis, Get_IMU_Yaw()); //
    ```
    
  * **四轮底盘**:

      * **手动控制**: 在 `4WIS/controllers/PID_Track/main.c` 中，直接调用 `Helm_Chassis_Ctrl`


    ```c
    // 示例: vx=0, vy=0, wz=90 (°/s)
    Helm_Chassis_Ctrl(0, 0, 90, &helm_chassis, Get_IMU_Yaw()); //
    ```

      * **轨迹跟踪**: 注释掉手动控制代码，并启用 `PID_Nav_Point_Handler` 和 `PID_Nav_Point_Tracker` 相关代码

    ```c
    // 启用轨迹跟踪
    PID_Nav_Point_Handler(&pid_nav,path_1);
    vel = PID_Nav_Point_Tracker(&pid_nav, &fzy_pid_x, &fzy_pid_y, &fzy_pid_v, path_1);
    vx = vel.vx;    
    vy = vel.vy;
    wz = vel.ang_w;
    Helm_Chassis_Ctrl(-vx/1000.0f, -vy/1000.0f, 0, &helm_chassis, Get_IMU_Yaw());
    ```

      * **修改路径**: 你可以编辑 `4WIS/controllers/PID_Track/path.c` 和 `path.h` 文件来定义自己的运动轨迹

## 👍 引用

如果你发现我的工作对你有帮助，请考虑引用我的论文：
```
@inproceedings{zhou2024fuzzy,
  title={Fuzzy-PID-based trajectory tracking for 3WIS robot},
  author={Zhou, Xingyu and Xu, Chaobin},
  booktitle={International Conference on Mechatronic Engineering and Artificial Intelligence (MEAI 2023)},
  volume={13071},
  pages={826--834},
  year={2024},
  organization={SPIE}
}
```

## 许可

该项目采用 MIT 许可。详情请见 `LICENSE` 文件。
