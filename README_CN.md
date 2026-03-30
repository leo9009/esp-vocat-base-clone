# ESP-VoCat 智能旋转底座

[English](README.md) | 简体中文

## 项目简介

ESP-VoCat 喵伴旋转底座是为[**ESP-VoCat 喵伴开发套件**](https://docs.espressif.com/projects/esp-dev-kits/zh_CN/latest/esp32s3/esp-vocat/user_guide_v1.2.html)量身打造的智能旋转底座，专用于玩具、智能音箱、智能中控等需要大模型语音交互能力的产品。设备采用 ESP32-C61-WROOM-1 模组，支持 USB Type-C 供电，并可通过磁吸接口为 ESP-VoCat 本体供电。

该项目实现了**高精度步进电机控制、磁吸滑动开关事件检测、 CSI 感知功能**、稳定的 **UART 通信**等核心功能，同时能够根据 ESP-VoCat 的 **声源识别（Sound Source Localization）** 结果自动调整方向，实现面向声源的智能旋转。通过动作控制与声源感知的结合，提供更**自然、更具沉浸感**的人机交互体验。

## 项目特点

- **精确角度控制** - 基于步进电机的高精度旋转定位
- **磁吸滑动开关** - 多种传感器支持，自动校准
- **丰富的预设动作** - 摇头、左顾右盼、鼓点律动等自然动作
- **自动归位** - 启动时自动寻找零位并校准
- **双向通信** - 主控板与底座实时数据交互
- **数据持久化** - 校准数据保存，掉电不丢失

## 项目结构

```
esp-vocat-base/
├── hardware/                    # 硬件设计文件
│   ├── schematics/              # 原理图
│   └── pcb/                     # PCB 设计文件
├── software/                    # 固件代码
│   ├── common_components/       # 四个工程共享组件
│   │   ├── stepper_motor/               # 步进电机控制
│   │   ├── magnetic_slide_switch/       # 磁吸开关（按示例选择 profile）
│   │   ├── control_serial/              # 串口通信
│   │   └── BMM150_SensorAPI/            # 传感器驱动
│   ├── esp_vocat_rotating_base/                         # 底座基础固件
│   ├── esp_vocat_rotating_base_bell_event_detection/    # 铃铛滑块事件检测示例
│   ├── esp_vocat_rotating_base_iphone_detection/        # 手机检测示例
│   └── esp_vocat_rotating_base_magnetic_accessory_detection/ # 磁吸配件检测示例
├── README.md                    # 英文说明文档
└── README_CN.md                 # 中文说明文档
```

## 工程示例说明

为方便在不切换分支的情况下体验不同功能，`software/` 下提供了 4 个可独立编译的示例工程：

- `esp_vocat_rotating_base`  
  基础版本，包含常规底座能力：步进电机动作、基础磁吸滑块事件、串口通信、校准与状态同步。

- `esp_vocat_rotating_base_bell_event_detection`  
  铃铛滑块事件检测示例，重点演示铃铛相关磁吸滑块事件识别逻辑。

- `esp_vocat_rotating_base_iphone_detection`  
  手机检测示例，重点演示手机靠近/离开及底座上下方相关检测事件。

- `esp_vocat_rotating_base_magnetic_accessory_detection`  
  磁吸配件检测示例，重点演示配件（如喂鱼/雪糕/甜甜圈等）相关磁场事件识别。

以上工程共享 `software/common_components/` 下的公共组件；其中 `magnetic_slide_switch` 按工程选择不同 profile 以兼容各示例的事件定义和检测策略。

## **功能展示**

[**视频展示**](https://www.bilibili.com/video/BV19gSEBwEoj/?spm_id_from=333.1365.list.card_archive.click&vd_source=e731e982043e3ccbb2c03395e0a66c39)

ESP-VoCat 底座采用乐鑫 **ESP32-C61-WROOM-1(N8R2)** 模组，通过自定义的 UART 协议，与 ESP-VoCat 本体进行串口通信。机械结构方面，底座配备 24BYJ48 步进电机，可实现声源识别和多种预定义旋转动作，增强交互体验。
此外，ESP-VoCat 底座配有一个磁吸滑动开关，可通过磁吸滑动开关，解锁更多交互体验。
### **声源识别**
ESP-VoCat 支持声源识别功能，能够实时检测环境中的声音方向和位置。系统通过麦克风阵列采集声音信号，结合声强、相位等信息进行分析，判断声源所在的方位角。配合基于步进电机的超静音旋转底座，可以实现面向声源的智能交互。  

![声源识别.gif](https://image.lceda.cn/oshwhub/pullImage/d77175cd4dd0484cb67eff6eaa344257.gif)

### **多种预设动作**
旋转底座支持多种预定义动作
- **摇头动作**
摇头动作让底座带动喵伴轻轻左右摆头，模拟猫咪好奇或轻微示意的动作。

![摇头.gif](https://image.lceda.cn/oshwhub/pullImage/d6aad38613fe448f9a7dd80cf92317be.gif)

- **好奇环顾**
好奇环顾动作让底座带动喵伴进行左顾右盼，配合小幅度随机偏移，模拟**猫咪自然观察环境**的行为。

![左顾右盼.gif](https://image.lceda.cn/oshwhub/pullImage/f5252b32170644bb90c492240ee94bf2.gif)

- **鼓点律动**
鼓点律动动作使 ESP-VoCat 能够根据外界音乐的**鼓点节奏**左右摆头，营造与音乐同步的**互动效果**。

![鼓点跟随.gif](https://image.lceda.cn/oshwhub/pullImage/f96c3a6d28774487ba1fabf218944f31.gif)

- **温柔蹭手**
温柔蹭手模拟猫咪轻柔蹭手的动作：底座缓慢扭向左侧再回到中心，循环数次。动作平滑自然，每次停顿都增强了真实的**触感**和**温柔感**。

![蹭手.gif](https://image.lceda.cn/oshwhub/pullImage/ca70534cecc342eea3f5e9ee2383a7a7.gif)

  **注意事项：**
  所有动作都可**参数化调控**，便于对接**豆包、小智**等大模型。
  而为更好执行动作，底座的电机控制拥有以下特性：
  - **精确角度控制：** 支持任意角度旋转，精度可达 0.1 度，服务于声源识别
  - **平滑加减速：** 采用加减速算法，旋转更自然流畅
  - **自动归位校准：** 
  由于 ESP-VoCat 旋转底座由步进电机驱动，因此没有角度反馈，故而底座在启动时将进行零位校准，校准流程如下：
   1. 上电后，电机会自动向左旋转寻找限位开关
   2. 触碰到限位开关后，电机会向右旋转 95° 到中心位置
   3. 校准完成后，电机断电
 
### **磁吸滑动开关交互控制**
ESP-VoCat 底座通过**磁吸式**滑动开关实现多种交互控制。滑块的不同位置会改变地磁传感器周围的磁场强度，底座通过实时监测这些磁场变化来识别滑块的动作。当检测到位置变化时，底座会将相应事件通过串口上报给 ESP-VoCat，从而实现丰富、直观的交互体验。

支持 9 种磁吸滑动开关事件检测：
   - 滑块自上向下拨动 (SLIDE_DOWN)
   - 滑块自下向上拨动 (SLIDE_UP)
   - 滑块从上方位置移除 (REMOVE_FROM_UP) 
   - 滑块从下方位置移除 (REMOVE_FROM_DOWN) 
   - 将滑块放置于上方位置 (PLACE_FROM_UP) 
   - 将滑块放置于下方位置 (PLACE_FROM_DOWN)
   - 当滑块处于下方位置时，可额外识别 **单击** 动作 (SINGLE_CLICK)：基于 mag_x 轴数据变化检测，结合 mag_x 和 mag_y 关系判断，防止滑动误触发
   - **喂鱼事件** (FISH_ATTACHED)：检测小鱼从上位置放置的特定磁场变化
   - **配对模式** (PAIRING)：两个 ESP-VoCat 面对面靠近，用于配对联网 

![磁吸开关.gif](https://image.lceda.cn/oshwhub/pullImage/351af8952a324622841299e806176f17.gif)

磁吸滑动开关控制功能支持**多种传感器**：
 - BMM150 三轴地磁传感器
 - QMC6309 三轴地磁传感器
 - 线性霍尔传感器

  **注意：**
  1. 以上传感器**三选一**即可，默认选择 **BMM150**。使用线性霍尔传感器时**只支持**滑块上下滑动（SLIDE_UP / SLIDE_DOWN）两个事件检测！
  2. 在首次使用该功能前，为适配磁场环境，须完成磁吸滑动开关的**全自动校准**，以记录其处于 **上方 / 下方 / 移除** 三种状态时的磁场强度基准值：
     - 系统自动检测三个稳定位置（每个位置保持静止约 500ms）
     - 无需手动操作提示，系统自动判断位置差异
     - 校准数据通过 NVS Flash 持久化存储，掉电不丢失
     - 在使用过程中如需重新校准，可通过 ESP-VoCat 发送串口命令或长按底座 Boot 键进行触发

依托地磁传感器的检测结果，结合智能动作判定算法，底座能够识别九类动作和事件。

### **CSI感知功能**
喵伴可通过 Wi-Fi CSI（Channel State Information）感知环境变化，实现动作触发或环境交互。
- **移动检测** 
通过喵伴获取路由器的CSI信息，可以感知整个空间环境内人体的移动，实现人体移动检测和数据统计。

- **近场感知** 
通过喵伴和底座之间互相发包，可以控制 Wi-Fi 传输路径，实现小范围高精度的检测效果。例如当检测到人手指靠近时，底座可自动调整姿态或执行预设动作，实现更智能的互动体验。

![C0482 (online-video-cutter.com).gif](https://image.lceda.cn/oshwhub/pullImage/54420a264bb946bb982edc07daac1b5c.gif)

### **串口通信**
ESP-VoCat 旋转底座可通过 **UART 协议**与 ESP-VoCat 进行数据通信。

- **命令类型**：
    - 角度控制命令（CMD_BASE_ANGLE_CONTROL）
    - 预设动作控制命令（CMD_BASE_ACTION_CONTROL）
    - 预设动作完成通知（CMD_ACTION_COMPLETE）
    - 磁吸开关事件上报（CMD_MAGNETIC_SWITCH_EVENT）
    - 磁吸数据校准进度同步（MAG_SWITCH_CALIB_*)

注：目前只开放以上基础的命令类型，后续会添加 CSI 相关命令。
    
-  **数据帧格式**:
    -   `AA 55 [LEN_H] [LEN_L] [CMD] [DATA...] [CHECKSUM]` 
-  **校验机制**：
    - **帧头**校验
    - **校验和**验证

[**详细的命令数据帧示例**](software/esp_vocat_rotating_base/README_CN.md)

## **硬件要求**

### **主控芯片**
- **ESP32-C61-WROOM-1** 或 **ESP32-C61-WROOM-1U** 模组

默认使用 ESP32-C61-WROOM-1

- [**硬件开源地址**](https://oshwhub.com/esp-college/esp-echoear-base)

