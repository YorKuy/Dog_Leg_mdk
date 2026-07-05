# 项目总结文档 — Dog_Leg_mdk (新底盘)

> **项目名称**: New_Chassis (新底盘)  
> **MCU**: STM32F405RGT6 (Cortex-M4, 168MHz)  
> **用途**: RoboMaster 机甲大师赛 — 双足腿部 + 麦轮底盘运动控制系统  
> **原始 IDE**: Keil MDK-ARM  
> **CMake 构建**: 已添加（不完整，编译失败）

---

## 一、项目总体概述

本项目是一个 RoboMaster 机器人的**底盘 + 腿部关节控制系统**。机器人底盘装有 4 个 M3508 麦轮电机，以及两个由达妙 (DM) 电机驱动的腿部关节，能够在多种模式下进行全向移动和姿态平衡。

### 硬件平台

| 项目 | 参数 |
|------|------|
| MCU 型号 | STM32F405RGT6 |
| 内核 | ARM Cortex-M4 (带 FPU) |
| 主频 | 168MHz (HSE 8MHz → PLL) |
| Flash | 1024KB |
| SRAM | 128KB (常规) + 64KB (CCMRAM) |

### 项目目录结构

```
Dog_Leg_mdk/
├── Core/                        # STM32CubeMX 生成的 HAL 层代码
│   ├── Inc/                     # 外设头文件 (can/spi/tim/usart/gpio/dma...)
│   └── Src/                     # 外设初始化 + main.c + ISR + sysmem/syscalls
├── Drivers/                     # STM32 官方驱动
│   ├── CMSIS/                   # CMSIS Core + Device 头文件
│   └── STM32F4xx_HAL_Driver/    # HAL 库源码
├── RM2023_Lib_V1.2/             # 机器人核心库 (最关键的模块)
│   ├── RM_Lib.cpp/h             # 遥控器DBUS、CAN封装、M3508/M2006电机驱动
│   ├── CP_System.c/h            # 裁判系统通讯协议(串口图形UI、状态数据)
│   ├── communication.c/h        # MiniPC/上位机串口通讯
│   └── my_math.c/h              # 数学工具函数
├── Motor_Lib/                   # 电机驱动封装
│   ├── DM.cpp/h                 # 达妙电机 (位置/速度/力矩 MIT 控制)
│   ├── RM.cpp/h                 # RoboMaster M3508/M2006 电机
│   └── RMD.cpp/h                # RMD 系列伺服电机
├── Math_Lib/                    # 控制算法库
│   ├── PID.cpp/h                # PID 控制器 (含模糊PID)
│   ├── SMC.cpp/h                # 滑模控制器 (3种实现)
│   └── Leg.cpp/h                # 腿部运动学
├── Control_Lib/                 # 腿部平衡控制
│   └── leg.cpp/h                # 双足腿部 SMC 平衡 + 前馈控制
├── IMU/                         # IMU 传感器
│   ├── imu_data_decode.c/h      # BMI088 数据解码
│   └── packet.c/h               # 数据包协议
├── CH010/                       # CH010 惯导模块
│   ├── hipnuc_dec.c/h           # HI226/HI229 惯导解码
│   ├── nmea_dec.c/h             # NMEA 协议解析
│   └── example_data.c/h         # 示例数据
├── cmake/                       # CMake 构建配置
│   ├── gcc-arm-none-eabi.cmake  # 交叉编译工具链
│   ├── starm-clang.cmake        # 备用工具链
│   └── stm32cubemx/CMakeLists.txt # CubeMX 生成源码配置
├── MDK-ARM/                     # Keil IDE 工程文件
│   └── New_Chassis.uvprojx      # Keil 项目文件
├── build/                       # CMake 构建输出
├── CMakeLists.txt               # 顶层 CMake 配置
├── CMakePresets.json            # CMake 预设
├── STM32F405XX_FLASH.ld         # 链接脚本
├── startup_stm32f405xx.s        # 启动汇编
├── New_Chassis.ioc              # CubeMX 项目配置
└── .mxproject                   # CubeMX 元数据
```

---

## 二、软件架构层次

```
┌──────────────────────────────────────────┐
│        应用控制层 (main.c)                │
│  模式调度 / 功率限制 / 底盘运动学 / UI     │
├──────────────────────────────────────────┤
│   Control_Lib (leg.cpp)                   │
│   腿部 SMC 平衡控制 / 前馈补偿             │
├──────────────────────────────────────────┤
│  Math_Lib (PID / SMC / Leg)               │
│  控制算法                                 │
├──────────────────────────────────────────┤
│  Motor_Lib (DM / RM / RMD)                │
│  电机驱动                                 │
├──────┬───────────────┬───────────────────┤
│ RM_Lib│ IMU / CH010   │ CP_System         │
│ DBUS  │ 传感器解码    │ 裁判系统通讯       │
│ CAN   │               │                   │
├──────┴───────────────┴───────────────────┤
│  Core + Drivers (HAL / CMSIS)             │
└──────────────────────────────────────────┘
```

---

## 三、外设配置一览

| 外设 | 引脚/通道 | 参数 | 用途 |
|------|-----------|------|------|
| HSE 晶振 | PH0/PH1 | 8MHz | 系统时钟源 |
| **CAN1** | PD0/PD1 | 1Mbps, BS1=9, BS2=4, Prescaler=3 | 底盘电机总线 (4×M3508 + 2×DM) |
| **CAN2** | PB5/PB6 | 1Mbps, 同上 | 云台/数控板通讯 |
| **SPI1** | PA5/PA6/PA7 | - | BMI088 陀螺仪 |
| **USART1** | PA9/PA10 | DMA2_Stream2(RX) + DMA2_Stream7(TX) | HI266 IMU 数据 |
| **USART2** | PD5/PD6 | - | 调试串口 (printf 输出) |
| **USART3** | PB10/PB11 | DMA1_Stream1(RX) + DMA1_Stream3(TX) | 裁判系统通讯 |
| **USART6** | PC6/PC7 | DMA2_Stream1(RX) | DBUS 遥控器接收 |
| **UART5** | PC12/PD2 | DMA1_Stream0(RX) + DMA1_Stream7(TX) | 额外串口 |
| **TIM2** | - | 25Hz 中断 | UI 客户端绘制刷新 |
| **TIM3** | - | 2kHz 中断 | 主控制循环 |
| **TIM5** | - | 100Hz 中断 | CAN 通讯调度 |
| **TIM7** | - | 定时器 | BMI088 采样基准 |
| **TIM8** | - | 定时器 | 额外 PWM 功能 |

---

## 四、核心控制逻辑

### 4.1 系统初始化流程

```
Reset → HAL_Init() → SystemClock_Config (168MHz)
  → MX_GPIO/DMA/CAN/SPI/TIM/UART_Init()
  → CAN_Motor.Init() + CAN_Communicate.Init()
  → Leg_Control_Config_MotorLimit()
  → CP_System_Init()
  → DM电机启动 (Left_Leg + Right_Leg)
  → 启动TIM中断 → while(1) 主循环
```

### 4.2 主循环逻辑

主循环每 0.5ms (2kHz) 执行一次，通过 TIM3 中断驱动：

```
TIM3 ISR (500μs 周期):
  ├─ 奇数周期:
  │   ├─ KEY_Forback_Ctrl1()      # 键盘前后左右控制
  │   ├─ Chassic_Forback_Ctrl()   # 摇杆底盘控制
  │   ├─ M3508_Limit_deal()       # 3508功率限制
  │   └─ CAN_Motor.Send_RM()      # 发送3508控制指令
  │
  └─ 偶数周期:
      ├─ LegControlInput 构造     # 组装控制输入
      ├─ Leg_SMC_Control()        # 腿部SMC平衡控制
      ├─ Left_Leg.DM_MIT()        # 左腿MIT控制
      └─ Right_Leg.DM_MIT()       # 右腿MIT控制
```

CAN2 中断服务 (100Hz):
```
TIM5 ISR:
  ├─ 发云台: Communicate_Send_Flag (0x558)
  └─ 发数控: 底盘功率/状态 (0x121)
```

### 4.3 遥控器工作模式 (8 种)

| 模式 | S1 | S2 | 行为 |
|------|----|----|------|
| PROTECT_MODE | UP | UP | 全失能，安全保护 |
| ONLY_GIMBAL | UP | MID | 仅控制云台 |
| ONLY_CHASSIC | MID | UP | 仅底盘移动 + 腿部陀螺仪闭环 |
| CONTROL_MODE | MID | MID | 底盘 + 腿部平衡控制 |
| XTL_MODE | DOWN | MID | 小陀螺旋转模式 |
| SHOOT_MODE | MID | DOWN | 射击模式 |
| PLAYER_MODE | DOWN | DOWN | 操作手模式 (完整功能) |
| FAST_CHASSIC | DOWN | UP | 快速底盘 + 腿部自动高度 |

### 4.4 腿部控制模式

通过 `leg.cpp` 中的 `Leg_SMC_Control()` 实现：

- **CONTROL/PLAYER/XTL 模式**: 使用滑模控制器 (SMC) 维持 Roll 轴平衡，通过两腿差动扭矩实现
- **FAST_CHASSIC 模式**: 高度位置控制，根据遥控器滚轮 (ch3) 调节腿部伸缩
- **其他模式**: 腿部失能，Torque = 0

### 4.5 功率限制策略

根据超级电容电压 V_Cap 分档限制输出功率：

- V_Cap ≥ 12V → 全功率 (KlimitGain = 1)
- 12V > V_Cap ≥ 10V → 比例限制
- 10V > V_Cap ≥ 8V → 平方限制
- V_Cap < 8V → 立方限制

按住 Shift 键可获得 1.5 倍限制系数。

---

## 五、依赖库说明

### RM2023_Lib_V1.2 (机器人核心库)

| 文件 | 说明 |
|------|------|
| `RM_Lib.h/cpp` | DBUS 遥控器解析、CAN 收发封装 (USER_CAN)、M3508/M2006 MOTOR_RM 类、BMI088 驱动、UpDown_check_class 边缘检测 |
| `CP_System.h/c` | RoboMaster 裁判系统完整数据结构和通讯协议：比赛状态、机器人状态、射击/弹丸数据、功率热量、客户端 UI 图形绘制 (线条/矩形/圆形/字符/数字) |
| `communication.h/c` | MiniPC 上位机串口通讯协议（自瞄/打符数据交换） |
| `my_math.h/c` | 数学工具：死区、限幅、PID 参数计算、循环检测等 |

### Motor_Lib (电机驱动)

| 文件 | 说明 |
|------|------|
| `DM.h/cpp` | 达妙电机完整驱动：MIT/位置/速度控制、多圈角度解算、电机状态 (温度/错误码) |
| `RM.h/cpp` | RoboMaster M3508/M2006 电机：速度转矩解析、CAN 发送 |
| `RMD.h/cpp` | RMD 伺服电机：PID 写入、编码器偏移、IQ/速度/角度控制 |

### Math_Lib (控制算法)

| 文件 | 说明 |
|------|------|
| `PID.h/cpp` | PID_class (位置式/增量式/低通滤波)、PID_Fuzzy_class (模糊自适应 PID) |
| `SMC.h/cpp` | 3 种滑模控制器：自研 MySMC、复旦版、川大版；包含 sign/saturate/boundaryLayer 函数 |
| `Leg.h/cpp` | 腿部运动学模型 |

### Control_Lib (腿部控制)

| 文件 | 说明 |
|------|------|
| `leg.h/cpp` | 双足机器人腿部平衡控制完整实现：SMC Roll 平衡、Pitch 补偿、加速度前馈、底盘加速度前馈、静态安静检测与扭矩保持、关节阻尼、积分死亡检测/复位 |

### 传感器驱动

| 文件 | 说明 |
|------|------|
| `IMU/packet.c/h` | 数据包帧协议 |
| `IMU/imu_data_decode.c/h` | BMI088 姿态数据解析 |
| `CH010/hipnuc_dec.c/h` | HI226/HI229 惯导模块解码 |
| `CH010/nmea_dec.c/h` | NMEA 协议解析 |
| `CH010/example_data.c/h` | 示例/测试数据 |

---

## 六、编译失败详细分析

### 6.1 配置阶段（通过）

CMake configure 成功，交叉编译工具链 `arm-none-eabi-gcc (GNU 10.3.1)` 检测正常。

### 6.2 编译阶段（失败）

#### 错误 #1

```
Core/Src/main.c:32:10: fatal error: RM_Lib.h: No such file or directory
   32 | #include "RM_Lib.h"
```

`main.c` 引用了 7 个自定义头文件，但编译器找不到它们：
- `RM_Lib.h` (位于 `RM2023_Lib_V1.2/`)
- `CP_System.h` (位于 `RM2023_Lib_V1.2/`)
- `DM.h` (位于 `Motor_Lib/`)
- `leg.h` (位于 `Control_Lib/`)
- `packet.h` (位于 `IMU/`)
- `imu_data_decode.h` (位于 `IMU/`)
- `hipnuc_dec.h` (位于 `CH010/`)

#### 错误 #2

```
Core/Src/stm32f4xx_it.c:25:10: fatal error: CP_System.h: No such file or directory
   25 | #include "CP_System.h"
```

同样的问题，`stm32f4xx_it.c` 引用了 `RM2023_Lib_V1.2/` 和 `IMU/` 等目录中的头文件。

### 6.3 根本原因

`cmake/stm32cubemx/CMakeLists.txt` 的配置**仅覆盖 STM32CubeMX 自动生成的代码**（HAL 驱动 + Core 层），完整缺失了 6 个用户自定义模块：

| 缺失项 | 详情 |
|--------|------|
| **Include 路径 (6 个目录)** | `RM2023_Lib_V1.2/`, `Motor_Lib/`, `Math_Lib/`, `IMU/`, `CH010/`, `Control_Lib/` |
| **源文件 (15+ 个)** | 上述目录中所有 `.c` / `.cpp` 文件 |

### 6.4 潜在二次问题

即使在 CMake 中添加了上述路径和源文件，还有以下潜在问题：

1. **C/C++ 混编**: `main.c` (C 文件) include 了 `DM.h`，后者定义了 C++ class `MOTOR_DM`。C 编译器无法解析 class 语法，会导致编译错误。原本在 Keil 中 `main.c` 可能被当作 C++ 编译。

2. **SMC.h 的 C++ STL 依赖**: `SMC.h` 包含了 `<cmath>`, `<algorithm>`, `<vector>`, `<memory>` 等 C++ 标准库头文件，需要嵌入式 C++ stdlib 支持。

3. **leg.cpp 中的构造函数**: [leg.cpp:78-90] 中 PID_class 和 SMC_PITCH 的静态初始化使用了带参数的构造函数，依赖 C++ 静态初始化机制。

---

## 七、文件统计

| 类别 | 文件数 | 目录 |
|------|--------|------|
| STM32 HAL 驱动 | ~120 | Drivers/STM32F4xx_HAL_Driver/ |
| CMSIS 核心 | ~50 | Drivers/CMSIS/ |
| CubeMX 生成 | 18 | Core/ |
| 机器人核心库 | 8 | RM2023_Lib_V1.2/ |
| 电机驱动 | 6 | Motor_Lib/ |
| 控制算法 | 6 | Math_Lib/ |
| 腿部控制 | 2 | Control_Lib/ |
| 传感器驱动 | 8 | IMU/ + CH010/ |
| 构建配置 | 5 | cmake/ + 根目录 |
| 总计 (源码) | ~40 用户文件 | - |

---

*文档生成日期: 2026-07-03*
