# 磁感应交互方案介绍

[中文](./magnetic_sensing_interaction_solution_zh.md) | [English](./magnetic_sensing_interaction_solution_en.md)

## 1. 功能概述

ESP-VoCat 底座通过磁吸式滑动开关实现多种交互控制。滑块不同位置会改变地磁传感器附近的磁场强度，系统通过持续采样与状态判定识别滑块动作，并将事件通过串口上报给 ESP-VoCat，实现更直观的交互体验。

## 2. 事件类型（按示例工程）

### 2.1 铃铛滑块事件检测示例
对应工程：[`software/esp_vocat_rotating_base_bell_event_detection`](./software/esp_vocat_rotating_base_bell_event_detection/)

支持事件：

- `SLIDE_DOWN`：滑块从上方拨到下方。
- `SLIDE_UP`：滑块从下方拨到上方。
- `REMOVE_FROM_UP`：滑块从上方位置被拿走。
- `REMOVE_FROM_DOWN`：滑块从下方位置被拿走。
- `PLACE_FROM_UP`：滑块被放置到上方位置。
- `PLACE_FROM_DOWN`：滑块被放置到下方位置。
- `SINGLE_CLICK`：滑块位于下方时触发一次单击动作。

### 2.2 手机检测示例
对应工程：[`software/esp_vocat_rotating_base_iphone_detection`](./software/esp_vocat_rotating_base_iphone_detection/)

支持事件：

- `IPHONE_LEAN_FRONT`：手机从底座正面倚靠上去。
- `IPHONE_LEAN_FRONT_DETACHED`：手机从底座正面离开。
- `IPHONE_UNDER_BASE`：手机被放到底座下方并进入有效检测状态。
- `IPHONE_UNDER_BASE_DETACHED`：手机从底座下方移开。

### 2.3 磁吸配件检测示例
对应工程：[`software/esp_vocat_rotating_base_magnetic_accessory_detection`](./software/esp_vocat_rotating_base_magnetic_accessory_detection/)

支持事件：

- `FISH_ATTACHED`：检测到喂鱼配件吸附。
- `FISH_DETACHED`：喂鱼配件从底座移除。
- `PAIRING`：检测到配对条件成立（进入配对状态）。
- `PAIRING_CANCELLED`：配对条件消失（退出配对状态）。
- `ICE_CREAM_ATTACHED`：检测到雪糕配件吸附。
- `ICE_CREAM_DETACHED`：雪糕配件从底座移除。
- `DONUT_ATTACHED`：检测到甜甜圈配件吸附。
- `DONUT_DETACHED`：甜甜圈配件从底座移除。

> 说明：以上事件来自三个示例工程对应的 `magnetic_slide_switch` profile 定义。实际以代码中的枚举为准。

## 3. 传感器支持

磁吸滑动开关支持以下传感器（三选一）：

- BMM150 三轴地磁传感器（默认）
- QMC6309 三轴地磁传感器
- 线性霍尔传感器

## 4. 使用注意事项

1. 不同示例工程事件集合不同，切换工程即可切换检测策略。
2. 首次使用前需执行全自动校准，记录上方 / 下方 / 移除三种状态的基准磁场值。
3. 校准数据会持久化保存至 NVS，掉电不丢失。
4. 运行中可通过串口命令或长按底座 Boot 键触发重新校准。
