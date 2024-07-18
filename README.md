# 启动方式

```bash
ros2 launch racing_state start.launch.py
python3 ./src/racing_pkg/racing_control/racing_control/racing_control.py
```

# 节点说明

## /hobot_usb_cam

### 发布话题

| 话题名 | 消息类型                        | 描述             |
| ------ | ------------------------------- | ---------------- |
| /image | sensor_msgs/msg/CompressedImage | 发布jpeg格式图片 |

## /hobot_codec

### 订阅话题

| 话题名 | 消息类型                        | 描述               |
| ------ | ------------------------------- | ------------------ |
| /image | sensor_msgs/msg/CompressedImage | 订阅摄像头发布图像 |

### 发布话题

| 话题名     | 消息类型                             | 描述             |
| ---------- | ------------------------------------ | ---------------- |
| /hbmem_img | rcl_interfaces/msg/HobotMemoryCommon | 发布nv12格式图片 |

## /body_det

### 订阅话题

| 话题名     | 消息类型                             | 描述             |
| ---------- | ------------------------------------ | ---------------- |
| /hbmem_img | rcl_interfaces/msg/HobotMemoryCommon | 订阅nv12格式图片 |

### 发布话题

| 话题名               | 消息类型                      | 描述         |
| -------------------- | ----------------------------- | ------------ |
| /hobot_dnn_detection | ai_msgs/msg/PerceptionTargets | 发布推理结果 |

## /line_point_detection

### 订阅话题

| 话题名 | 消息类型                        | 描述             |
| ------ | ------------------------------- | ---------------- |
| /image | sensor_msgs/msg/CompressedImage | 订阅jpeg格式图片 |

### 发布话题

| 话题名                | 消息类型                | 描述             |
| --------------------- | ----------------------- | ---------------- |
| /line_point_detection | origincar_msg/msg/Point | 发布赛道中点信息 |

## /line_point_detection

### 订阅话题

| 话题名 | 消息类型                        | 描述             |
| ------ | ------------------------------- | ---------------- |
| /image | sensor_msgs/msg/CompressedImage | 订阅jpeg格式图片 |

### 发布话题

| 话题名                | 消息类型                        | 描述               |
| --------------------- | ------------------------------- | ------------------ |
| /line_point_detection | origincar_msg/msg/Point         | 发布赛道中点信息   |
| /process_image        | sensor_msgs/msg/CompressedImage | 赛道中点识别可视化 |

## /racing_control

### 订阅话题

| 话题名                | 消息类型                      | 描述           |
| --------------------- | ----------------------------- | -------------- |
| /hobot_dnn_detection  | ai_msgs/msg/PerceptionTargets | 订阅障碍物信息 |
| /line_point_detection | origincar_msg/msg/Point       | 订阅中点信息   |

### 发布话题

| 话题名   | 消息类型                | 描述         |
| -------- | ----------------------- | ------------ |
| /cmd_vel | geometry_msgs/msg/Twist | 发布速度信息 |

## /qrdecoder

### 订阅话题

| 话题名 | 消息类型                        | 描述             |
| ------ | ------------------------------- | ---------------- |
| /image | sensor_msgs/msg/CompressedImage | 订阅jpeg格式图片 |

### 发布话题

| 话题名         | 消息类型               | 描述           |
| -------------- | ---------------------- | -------------- |
| /sign_foxglove | origincar_msg/msg/Sign | 发布上位机信号 |
| /sign_switch   | origincar_msg/msg/Sign | 发布小车信号   |

## /racing_state

### 订阅话题

| 话题名       | 消息类型           | 描述               |
| ------------ | ------------------ | ------------------ |
| /sign4return | std_msgs/msg/Int32 | 订阅上位机按键信号 |

### 发布话题

| 话题名         | 消息类型               | 描述           |
| -------------- | ---------------------- | -------------- |
| /sign_foxglove | origincar_msg/msg/Sign | 发布上位机信号 |

