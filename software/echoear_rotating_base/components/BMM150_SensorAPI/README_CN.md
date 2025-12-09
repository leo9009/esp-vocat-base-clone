# BMM150 磁力计 AUX 接口适配器

## 概述

本组件通过 BMI270 的 AUX 接口实现 BMM150 磁力计功能。它允许通过 BMI270 的辅助 I2C 接口直接访问 BMM150 寄存器，使得在 BMM150 通过 BMI270 连接的系统中能够采集磁力计数据。

## 项目结构

```
LLM_camera/
├── examples/bmi270_bmm150/
│   ├── main/
│   │   ├── main.c                    # 主程序入口
│   │   ├── CMakeLists.txt            # 构建配置
│   │   └── idf_component.yml         # 组件依赖
│   └── components/
│       └── BMM150_SensorAPI/         # BMM150 组件库
│           ├── bmm150_aux_adapter.h  # AUX 适配器头文件
│           ├── bmm150_aux_adapter.c  # AUX 适配器实现
│           ├── bmm150_aux_example.c  # 使用示例
│           ├── bmm150.h              # 原始 BMM150 驱动头文件
│           ├── bmm150.c              # 原始 BMM150 驱动实现
│           ├── bmm150_defs.h         # BMM150 定义和常量
│           ├── test_apps/            # 测试应用
│           ├── examples/             # 示例代码
│           ├── CMakeLists.txt        # 组件构建配置
│           ├── idf_component.yml     # 组件依赖
│           ├── README.md             # 英文文档
│           ├── CHANGELOG.md          # 版本历史
│           └── LICENSE               # 许可证信息
```

## 调用关系

### 1. 主程序 (main.c)
- **入口函数**：`app_main()`
- **初始化流程**：
  1. `i2c_sensor_bmi270_init()` - 初始化 I2C 总线和 BMI270 驱动
  2. `bmi270_init_and_config()` - 初始化 BMI270 并配置 AUX 接口
  3. `bmm150_init_and_config()` - 通过 AUX 适配器初始化 BMM150
  4. `enable_sensors()` - 使能所有传感器（加速度、陀螺仪、磁力计）

### 2. BMM150 AUX 适配器 (bmm150_aux_adapter.c)
- **核心函数**：
  - `bmm150_aux_adapter_init()` - 通过 AUX 接口初始化 BMM150
  - `bmm150_aux_adapter_read_mag_data()` - 读取磁力计数据
  - `bmm150_aux_adapter_configure()` - 配置 BMM150 参数
  - `bmm150_aux_adapter_calculate_heading()` - 计算磁力计航向角
  - `bmm150_aux_adapter_is_field_normal()` - 检查磁场有效性

### 3. BMI270 组件
- **AUX 接口**：为 BMM150 提供 I2C 桥接
- **数据采集**：采集加速度、陀螺仪和磁力计数据
- **传感器融合**：支持多传感器数据融合

## 硬件配置

### 支持的硬件
- **ESP-ASTOM-S3**：I2C 引脚 0/45，INT 引脚 16

### I2C 配置
- **主模式**：I2C_NUM_0
- **频率**：100kHz
- **上拉**：SDA 和 SCL 启用上拉
- **软件 I2C**：可选（UE_SW_I2C 标志）

## 正反面方向适配（Face Up/Down Orientation Correction）

### 问题描述
当带有磁力计的设备被翻转（正面朝上/朝下）时，磁力计读数会反向，但航向角计算需要保持一致。这对于设备可能正反两面使用的场景尤为重要。

### 解决方案实现

#### 1. Z 轴朝向检测
```c
/* 使用加速度计判断当前 Z 轴朝向 */
float acc_z_mg = (float)sensor_data.acc.z / 16384.0f;
int current_z_sign = (acc_z_mg > 0) ? 1 : -1;
```

#### 2. 朝向变化检测
```c
/* 检测 Z 轴朝向变化 */
if (last_z_sign != 0 && current_z_sign != last_z_sign) {
    if (current_z_sign > 0) {
        ESP_LOGI(TAG, "Z axis flipped: Now facing UP (acc_z_mg=%.2f)", acc_z_mg);
    } else {
        ESP_LOGI(TAG, "Z axis flipped: Now facing DOWN (acc_z_mg=%.2f)", acc_z_mg);
    }
}
```

#### 3. 航向修正算法
```c
/* 正反面方向适配 */
float corrected_heading = heading;
if (current_z_sign > 0) {
    /* 正面朝上：用 360 - heading 修正 */
    /* 因为翻转时磁力计读数反向，需修正航向角保证一致性 */
    corrected_heading = 360.0f - heading;
    if (corrected_heading >= 360.0f) corrected_heading -= 360.0f;
} else {
    /* 正面朝下：直接用原始航向角 */
}
```

### 技术细节

#### 为什么需要修正
1. **磁力计反向**：设备翻转时，X/Y 轴磁力计读数反向
2. **航向计算**：`atan2(mag_y, mag_x)` 对反向读数结果不同
3. **一致性需求**：无论设备正反，物理方向显示的航向应一致

#### 修正逻辑
- **正面朝下（Z 轴向下）**：直接用原始航向角
- **正面朝上（Z 轴向上）**：用 `360° - heading` 修正
- **效果**：同一物理方向始终显示相同航向

#### 数学推导
- **原始航向**：`θ = atan2(mag_y, mag_x)`
- **反向读数**：`mag_x' = -mag_x`, `mag_y' = -mag_y`
- **反向航向**：`θ' = atan2(-mag_y, -mag_x) = atan2(mag_y, mag_x) + 180°`
- **修正**：`θ_corrected = 360° - θ' = 360° - (θ + 180°) = 180° - θ`

### 使用示例
```c
/* 结合 Z 轴朝向自动修正航向角 */
float heading = bmm150_aux_adapter_calculate_heading(mag_x, mag_y);
float acc_z_mg = (float)sensor_data.acc.z / 16384.0f;

/* 判断当前 Z 轴朝向 */
int current_z_sign = (acc_z_mg > 0) ? 1 : -1;

/* 正反面方向适配 */
float corrected_heading = heading;
if (current_z_sign > 0) {
    /* 正面朝上：用 360 - heading 修正 */
    corrected_heading = 360.0f - heading;
    if (corrected_heading >= 360.0f) corrected_heading -= 360.0f;
} else {
    /* 正面朝下：直接用原始航向角 */
}

/* 显示方向和角度 */
const char* direction = bmm150_aux_adapter_get_direction(corrected_heading);
ESP_LOGI(TAG, "Heading: %s (%.1f)", direction, corrected_heading);
```

## 数据处理特性

### 运动检测
- **阈值**：加速度变化 50mg
- **监控轴**：X、Y、Z
- **输出**：运动检测提示

### 旋转检测
- **阈值**：角速度变化 10 度/秒
- **监控轴**：X、Y、Z
- **输出**：旋转方向识别

### 磁场监控
- **强度计算**：`sqrt(mag_x² + mag_y² + mag_z²)`
- **正常范围**：20-100 μT
- **变化检测**：10 μT 阈值
- **输出**：磁场强度和有效性

### 航向计算
- **方法**：`atan2(mag_y, mag_x)`
- **范围**：0-360 度
- **方向映射**：北（315-45°）、东（45-135°）、南（135-225°）、西（225-315°）
- **正反面适配**：自动修正

## 错误处理

### 初始化错误
- **I2C 总线创建**：检查总线创建失败
- **BMI270 初始化**：校验芯片 ID 和通信
- **BMM150 初始化**：校验芯片 ID 和 AUX 接口
- **传感器配置**：校验参数应用

### 运行时错误
- **数据读取失败**：处理传感器通信错误
- **无效数据**：检查磁场强度异常
- **朝向变化**：检测并处理设备翻转

## 性能考量

### 时序
- **数据速率**：磁力计 10Hz，加速度/陀螺仪 100Hz
- **处理延迟**：方向修正开销极小
- **内存占用**：静态变量跟踪朝向

### 功耗
- **正常模式**：BMM150 处于正常功耗模式
- **AUX 接口**：功耗极低
- **处理**：高效数学运算

## 故障排查

### 常见问题
1. **无磁力计数据**：检查 AUX 接口配置
2. **航向不正确**：校验磁场强度和朝向
3. **通信错误**：检查 I2C 连接和地址
4. **朝向异常**：确保加速度计数据有效

### 调试信息
- **原始数据**：磁场变化时可用
- **芯片 ID**：初始化时校验
- **朝向变化**：检测到时日志输出
- **磁场强度**：持续监控

## 许可证

本组件遵循 Apache-2.0 许可证。详见 LICENSE 文件。

## 版本历史

详细版本历史和变更请见 CHANGELOG.md。 