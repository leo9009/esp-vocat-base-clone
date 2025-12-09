# ESP-EchoEar-Base: 喵伴旋转底座

**中文** | [English](README.md)

## 项目简介

该项目工程是乐鑫"喵伴"智能交互设备的旋转底座固件，基于 ESP32-C61 芯片开发。该项目实现了步进电机精确控制、磁吸滑动开关检测、串口通信等功能，为智能设备提供生动的交互动作和灵敏的输入检测。

## 主要特性

### 步进电机控制
- **多种预定义动作**：
  - 摇头动作（固定幅度/渐变递减）
  - 左顾右盼观察动作
  - 猫咪蹭手动作
  - 跟随鼓点左右摆头动作

所有动作都可参数化调控，便于对接豆包、小智等大模型。
- **精确角度控制**：支持任意角度旋转，精度可达 0.1 度
- **平滑加减速**：采用加减速算法，运动更自然流畅
- **半步驱动模式**：提高精度 2 倍，运动更平滑
- **自动归位校准**：启动时通过限位开关自动回零

### 磁吸滑动开关
- **多种传感器支持**：
  - BMM150 三轴地磁传感器
  - QMC6309 三轴地磁传感器
  - 线性霍尔传感器

以上三种传感器三选一即可，默认选择 BMM150。
- **丰富的事件检测**：
  - 滑块上下滑动（SLIDE_UP / SLIDE_DOWN）
  - 从上/下位置取下（REMOVE_FROM_UP / REMOVE_FROM_DOWN）
  - 从上/下位置放回（PLACE_FROM_UP / PLACE_FROM_DOWN）
  - 单击事件（SINGLE_CLICK）
- **智能校准功能**：
  - 首次使用自动引导校准
  - 校准数据持久化存储（NVS Flash）
  - 支持串口命令触发重新校准
  - 支持长按 Boot 键触发重新校准
  - 自动适配不同磁场环境

### 串口通信
- **UART 协议通信**：与 EchoEar 进行数据通信
- **命令类型**：
  - 角度控制命令（CMD_BASE_ANGLE_CONTROL）
  - 预设动作控制命令（CMD_BASE_ACTION_CONTROL）
  - 预设动作完成通知（CMD_ACTION_COMPLETE）
  - 磁吸开关事件上报（CMD_MAGNETIC_SWITCH_EVENT）
  - 校准进度通知（MAG_SWITCH_CALIB_*）
- **数据帧格式**：`AA 55 [LEN_H] [LEN_L] [CMD] [DATA...] [CHECKSUM]`
- **校验机制**：支持**帧头**校验、**校验和**验证

## 硬件要求

### 主控芯片
- **ESP32-C61-WROOM-1** 或 **ESP32-C61-WROOM-1U** 模组

默认使用 ESP32-C61-WROOM-1

### 外设连接与引脚分配

#### 步进电机（28BYJ-48 + ULN2003 驱动 IC）
- IN1 → GPIO28
- IN2 → GPIO27
- IN3 → GPIO26
- IN4 → GPIO25

#### 磁力计传感器（I2C）
- SCL → GPIO3
- SDA → GPIO2
- 支持 BMM150（I2C 地址：0x10）
- 支持 QMC6309（I2C 地址：0x7C）

#### 线性霍尔传感器（ADC）
- ADC_PIN → GPIO5（ADC_CHANNEL_3）

#### 转台回零校准限位开关
- 回零校准限位开关检测引脚 → GPIO1（低电平有效）

#### BOOT 按键
- BOOT 按键 → GPIO9（低电平有效）

默认用于长按进行磁吸滑动开关校准。

#### UART 通信
- TXD → GPIO29
- RXD → GPIO8
- 波特率：115200

## 软件架构

### 主要组件

#### 1. stepper_motor（步进电机控制）
- 实现半步驱动控制算法
- 提供加减速曲线
- 封装多种预定义动作
- 支持精确角度旋转

#### 2. magnetic_slide_switch（磁吸滑动开关）
- 滑动窗口滤波算法
- 状态机实现事件检测
- 单击检测（峰值检测法）
- 自动校准流程
- 校准数据 NVS 数据存储

#### 3. control_serial（串口通信）
- UART 收发任务
- 协议帧解析
- 命令分发处理
- 事件上报

#### 4. BMM150_SensorAPI（传感器驱动）
- BMM150 官方 API 适配
- I2C 总线封装

### 任务结构
```
app_main
├── base_calibration_task     // 底座角度校准任务（启动时）
├── uart_cmd_receive_task     // UART 命令接收任务
├── uart_cmd_send_task        // UART 命令发送任务
└── magnetometer_read_task    // 磁力计读取任务
```

## 快速开始

### 环境准备

1. **ESP-IDF 版本**

- ESP-IDF release/v5.5(v5.5-445-g1191f4c4f91-dirty)

### 配置项目

使用 `idf.py menuconfig` 选择磁吸滑动开关所用的传感器：

 `Component config → Magnetic Slide Switch → Sensor Type`

选择传感器类型：BMM150 / QMC6309 / Linear Hall Sensor

### 编译烧录

```bash
# 编译项目
idf.py build
# 烧录固件
idf.py flash
# 查看日志
idf.py monitor
```

## 使用说明

### 首次启动

1. **底座角度校准**：
   - 上电后，电机会自动向左旋转寻找限位开关
   - 触碰到限位开关后，电机会向右旋转 95° 到中心位置
   - 校准完成后，电机断电

2. **磁吸开关校准**（如果是首次使用）：
   - 系统提示：`Please keep the slider in current position and wait for calibration...`，并发送 `0x11` 给 EchoEar，通知 EchoEar 校准开始
   - 保持滑块在当前位置不动，等待数据稳定（约 1 秒）
   - 系统记录第一个位置后，提示：`Please move the slider to another position...`
   - 将滑块移动到另一个位置（上/下/拿掉），等待数据稳定
   - 系统记录第二个位置后，发送 `0x12` 给 EchoEar，EchoEar 语音提示第三步
   - 将滑块移动到第三个位置，等待数据稳定
   - 系统记录第三个位置后，校准完成，发送 `0x13` 给 EchoEar，通知 EchoEar 校准完毕
   - 校准数据自动保存到 Flash，下次启动自动加载

### 重新校准

**方法 1：长按 Boot 按键**
- 长按 GPIO9 的 Boot 按键（约 1.5 秒）
- 系统自动进入重新校准模式

**方法 2：串口命令**
- 发送命令：`AA 55 00 03 03 00 10 13`
- 系统收到命令后进入重新校准模式

### 串口命令

#### 1. 磁吸通知命令（0x00）
`
AA 55 00 03 00 00 01 01
`

底座会周期性发送该命令，EchoEar 在被磁吸连接后若成功接收到此命令，即可确认已与底座建立连接，并开始播放连接动画。

#### 2. 角度控制命令（0x01）
**格式**：`AA 55 00 03 01 [ANGLE_H] [ANGLE_L] [CHECKSUM]`

**示例**：旋转到 45°
```
AA 55 00 03 01 00 2D 2E
```
- `00 2D` = 45（十进制）
- `2E` = `01 + 00 + 2D`（校验和）

#### 3. 动作控制命令（0x02）
**格式**：`AA 55 00 03 02 00 [ACTION] [CHECKSUM]`

**动作码**：
- `0x01`：摇头（SHAKE_HEAD）
- `0x02`：左顾右盼（LOOK_AROUND）
- `0x03`：跟随鼓点-鼓类（BEAT_SWING_DRUM）
- `0x04`：猫咪蹭手（CAT_NUZZLE）
- `0x05`：跟随鼓点-其他乐器（BEAT_SWING_OTHER）

**示例**：执行左顾右盼动作
```
AA 55 00 03 02 00 02 04
```

#### 4. 磁吸开关事件上报（0x03）
**格式**：`AA 55 00 03 03 00 [EVENT] [CHECKSUM]`

**事件码**：
- `0x01`：滑块向下滑动
- `0x02`：滑块向上滑动
- `0x03`：从上面拿下
- `0x04`：从下面拿下
- `0x05`：从上面放回
- `0x06`：从下面放回
- `0x07`：单击事件
- `0x11`：第二个位置校准完成（头部收到后语音提示）
- `0x12`：校准完成

#### 5. 动作完成通知（0x02）
**格式**：`AA 55 00 03 02 00 10 12`

- 底座在执行完任意特定动作后自动发送
- 头部可根据此命令同步状态

## 步进电机动作详解

### 1. stepper_shake_head（摇头）
```c
stepper_shake_head(float amplitude, int cycles, int speed_us);
```
- `amplitude`：摆动幅度（度）
- `cycles`：摆动次数
- `speed_us`：速度（微秒/步）

### 2. stepper_shake_head_decay（渐变摇头）
```c
stepper_shake_head_decay(float initial_amplitude, float decay_rate, int speed_us);
```
- `initial_amplitude`：初始幅度（度）
- `decay_rate`：衰减系数（0.0-1.0，如 0.8 表示每次衰减 20%）
- `speed_us`：速度（微秒/步）

### 3. stepper_look_around（左顾右盼）
```c
stepper_look_around(float left_angle, float right_angle, float scan_angle, 
                    int pause_ms, int large_speed_us, int small_speed_us);
```
- `left_angle`：左侧最大角度
- `right_angle`：右侧最大角度
- `scan_angle`：小幅度扫视角度
- `pause_ms`：每次转动后的停顿时间
- `large_speed_us`：大幅度转动速度
- `small_speed_us`：小幅度扫视速度

### 4. stepper_cat_nuzzle（猫咪蹭手）
```c
stepper_cat_nuzzle(float angle, int cycles, int speed_us);
```
- `angle`：蹭动幅度（度）
- `cycles`：蹭动次数
- `speed_us`：速度（微秒/步）

### 5. stepper_beat_swing（跟随鼓点）
```c
stepper_beat_swing(float angle, int speed_us);
```
- `angle`：摆动角度（度）
- `speed_us`：速度（微秒/步）
- **智能摆动逻辑**：
  - 同类乐器连续触发 → 向另一侧摆动 15 度

## 磁吸开关校准原理

### 校准流程状态机

```
CALIBRATION_FIRST_POSITION    // 等待第一个位置稳定
         ↓                    // （发送 0x11 通知头部）
CALIBRATION_WAIT_SECOND       // 等待移动到第二个位置
         ↓                    // （检测到移动后进入沉淀期）
CALIBRATION_SECOND_POSITION   // 等待第二个位置稳定
         ↓                    // （发送 0x12 通知头部）
CALIBRATION_WAIT_THIRD        // 等待移动到第三个位置
         ↓                    // （检测到移动后进入沉淀期）
CALIBRATION_THIRD_POSITION    // 等待第三个位置稳定
         ↓
CALIBRATION_COMPLETED         // 自动排序并保存到 Flash
                              // （发送 0x13 通知头部）
```

### 关键技术点

1. **滑动窗口滤波**：5 点滑动平均，降低噪声干扰
2. **稳定性检测**：连续多次（15 次）数值变化小于阈值才认为稳定
3. **沉淀期机制**：检测到移动后，先等待数据完全稳定（约 500ms）再记录
4. **自动排序**：校准完成后按磁场强度自动分配到 REMOVED/UP/DOWN
5. **NVS 持久化**：校准数据保存到 Flash，掉电不丢失

### 磁场值参考范围（BMM150）
- **REMOVED**（拿掉）：~1230 ± 100
- **UP**（上位置）：~1619 ± 100
- **DOWN**（下位置）：~1894 ± 100

*注：实际值因磁铁强度和安装位置而异，需要校准*

### 常见问题

**Q1：电机抖动或不转**
- 检查供电是否充足（至少 5V 1A）
- 检查接线是否正确
- 尝试降低速度（增大 `speed_us` 参数）

**Q2：磁吸开关校准失败**
- 确保传感器 I2C 连接正常
- 检查传感器类型配置是否匹配
- 移动滑块时动作要明确，等待稳定提示再移动
- 慢速移动时，检查是否已优化沉淀期机制

**Q3：串口通信异常**
- 检查波特率是否为 115200
- 确认 TX/RX 接线是否正确
- 检查校验和是否计算正确

**Q4：首次上电电机旋转方向错误**
- 检查限位开关安装位置
- 调整 `base_calibration_task` 中的旋转角度

## 性能参数

- **角度控制精度**：±0.5°
- **最快旋转速度**：600 μs/步（约 200°/秒）
- **磁吸开关响应时间**：< 50ms
- **磁吸开关采样率**：200 Hz（BMM150）/ 500 Hz（QMC6309）
- **串口通信速率**：115200 bps
- **内存占用**：~80KB RAM，~200KB Flash

## 版本历史

### v1.0.0
- 步进电机基础控制
- 多种预定义动作
- 磁吸滑动开关事件检测
- 自动校准功能
- NVS 数据持久化
- 串口通信协议
- 智能跟随鼓点算法
- 长按 Boot 键重新校准
- 校准过程串口通信
- 慢速移动校准优化

## 许可证

本项目遵循 GPL 3.0 许可证。

---

**注意**：本项目为乐鑫"喵伴"智能设备配套固件，仅供学习和参考使用。
