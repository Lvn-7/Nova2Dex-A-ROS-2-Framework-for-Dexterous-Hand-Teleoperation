# Nova2 UDP 协议（version 3）

Windows 端 `nova2_windows_udp_reader.exe` 以 UDP datagram 发送 UTF-8 JSON。
每个 datagram 是一帧数据，默认发送频率为 60 Hz；一帧可以包含左手、右手或两只手。

## 传输

```text
协议：UDP
编码：UTF-8
方向：Windows reader -> Ubuntu bridge
默认端口：15020
```

UDP 不提供 ACK、重传或顺序保证。接收端应以最新收到的完整帧为准。

## 顶层格式

```json
{
  "source": "senseglove_nova2",
  "version": 3,
  "timestamp_ms": 123456789,
  "hands": [
    { "...": "one hand frame" }
  ]
}
```

字段说明：

| 字段 | 类型 | 说明 |
|---|---|---|
| `source` | string | 固定为 `senseglove_nova2` |
| `version` | integer | 当前为 `3` |
| `timestamp_ms` | integer | Windows `steady_clock` 时间，适合比较帧间隔，不是 Unix 时间戳 |
| `hands` | array | 当前帧中的手套数据，最多两项 |

## 单手格式

```json
{
  "side": "right",
  "glove_id": 1,
  "connected": true,
  "is_right": true,
  "sensor_data_valid": true,
  "sensor_channels": {
    "thumb_flexion": 0,
    "index_flexion_proximal": 0,
    "index_flexion_distal": 0,
    "middle_flexion": 0,
    "ring_flexion": 0,
    "thumb_abduction": 0
  },
  "imu_orientation_xyzw": [0, 0, 0, 1],
  "imu_valid": true,
  "hand_angles_rad": [
    [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
  ],
  "joint_positions_mm": [
    [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
  ],
  "joint_rotations_xyzw": [
    [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]],
    [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]],
    [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]],
    [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]],
    [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]]
  ],
  "normalized_flexion": [0, 0, 0, 0, 0],
  "battery_level": 0.85,
  "battery_valid": true,
  "is_charging": false
}
```

### 设备字段

| 字段 | 类型 | 顺序/取值 |
|---|---|---|
| `side` | string | `left` 或 `right` |
| `glove_id` | integer | 左手 `0`，右手 `1` |
| `connected` | bool | 成功取得 `HandPose` 时为 `true` |
| `index_influence_others` | bool | 从 HandLayer 实际手套实例读回的食指耦合开关，当前 GUI 默认关闭；设置或姿态读取后校验失败时不发送该手数据 |
| `is_right` | bool | 右手 `true`，左手 `false` |
| `sensor_data_valid` | bool | 底层传感器数据是否成功取得 |
| `battery_level` | float | `0..1`；读取失败时发送 `-1` |
| `battery_valid` | bool | 电量读取是否成功 |
| `is_charging` | bool | 是否正在充电 |

### 底层六路传感器

`sensor_channels` 是 reader 通过 SDK 的按名称接口
`GetSensorValue(finger, sensor_location)` 读取的 6 个标量。这里使用对象字段而不是数组，避免把 SDK 未承诺的内部原始报文索引伪装成协议顺序：

```text
sensor_channels.thumb_flexion           -> Thumb / FlexionProximal
sensor_channels.index_flexion_proximal -> Index / FlexionProximal
sensor_channels.index_flexion_distal   -> Index / FlexionDistal
sensor_channels.middle_flexion         -> Middle / FlexionProximal
sensor_channels.ring_flexion           -> Ring / FlexionProximal
sensor_channels.thumb_abduction        -> Thumb / Abduction
```

数值单位和原始量纲由 SenseGlove SDK 的 `Nova2GloveSensorData` 定义；该字段不是归一化弯曲值。
Nova 2 没有独立的小拇指物理传感器，小拇指屈曲由 SDK 根据无名指传感器关联/推导；因此 `sensor_channels` 中没有小拇指独立通道。

SDK 头文件提供了按手指和位置读取的接口，但没有在本地头文件中承诺“原始六路报文”的数组索引排列。因此接收端必须按本协议的字段语义解释，不能把它当作 SenseCom 原始报文的第 0～5 号通道。

### IMU

| 字段 | 类型 | 顺序/单位 |
|---|---|---|
| `imu_orientation_xyzw` | array[4] | 四元数顺序 `[x, y, z, w]`，表示手套三自由度旋转，不含位置 |
| `imu_valid` | bool | IMU 是否成功读取 |

### 关节数据

以下数组统一采用“拇指、食指、中指、无名指、小指”的手指顺序。

| 字段 | 维度 | 顺序 | 单位 |
|---|---:|---|---|
| `hand_angles_rad` | `[5][3][3]` | 手指 → 关节 → Euler 分量 XYZ | rad |
| `joint_positions_mm` | `[5][4][3]` | 手指 → 节点 → XYZ | mm |
| `joint_rotations_xyzw` | `[5][4][4]` | 手指 → 节点 → 四元数 XYZW | 无量纲 |
| `normalized_flexion` | `[5]` | 手指 | `0..1` |

关节/节点的具体语义由 SenseGlove SDK 的 `HandPose` 定义；数组位置顺序不在 UDP 层重新排列。

## 左右手和双手

Windows reader 参数控制当前帧包含哪些手：

```text
--left true  --right false   只发送左手
--left false --right true    只发送右手
--left true  --right true    同一帧发送双手
```

双手时 `hands` 数组中分别包含 `side=left` 和 `side=right` 的对象，接收端不应仅依赖数组下标判断左右手，应使用 `side` 或 `is_right`。
