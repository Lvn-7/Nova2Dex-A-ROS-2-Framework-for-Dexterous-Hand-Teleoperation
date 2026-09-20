
# Inspire G1 双 Inspire 灵巧手服务

当前版本面向 Unitree G1，以及三类测试程序：官方 `hand_example`、整手开合测试和单指测试。

## 工作方式

两只手连接在同一个 RS-485 总线上，服务默认优先通过 `/dev/serial/by-id`
自动寻找唯一的 FTDI 设备，再回退到唯一的 `/dev/ttyUSB*`。服务通过设备 ID 区分左右手：

```text
自动检测的 FTDI 串口
├── ID 1：左手
└── ID 2：右手
```

服务使用独立的 DDS 命令主题，向一只手下发命令不会给另一只手下发命令：

```text
rt/inspire/left/cmd
rt/inspire/right/cmd
```

每个主题包含该手的 6 个命令。命令字段为：

- `q`：位置，归一化范围 `0~1`
- `dq`：速度，归一化范围 `0~1`

服务端将位置和速度分别转换为 Inspire 协议的 `0~1000`。当前 Inspire 的位置含义是：

```text
0：闭合
1：张开
```

速度 `0` 最慢，`1` 为当前接口允许的最大归一化速度。

## 手指顺序

左右手顺序相同：

```text
0：小拇指
1：无名指
2：中指
3：食指
4：拇指弯曲
5：拇指旋转
```

## 编译

```bash
sudo apt install libboost-all-dev libeigen3-dev
cmake -S . -B build
cmake --build build -j6
```

如果 SDK 头文件或库缺失，请先编译安装 `unitree_sdk2`。

## 启动服务

```bash
sudo ./build/inspire_g1
```

如有多个串口候选，服务会拒绝猜测；请明确指定：

```bash
sudo ./build/inspire_g1 --serial /dev/serial/by-id/<device-id>
```

服务需要 root 权限访问 `/dev/ttyUSB0`，测试程序通过 DDS 工作，不需要 `sudo`。服务和测试程序不要同时打开串口；测试程序只连接 DDS，因此可以在服务运行时使用。

## 测试程序

### 整手开合

```bash
./build/hand_all_test left open
./build/hand_all_test left close
./build/hand_all_test right open
./build/hand_all_test right close
```

### 单指位置和速度

```bash
./build/hand_finger_test <left|right> <手指序号> <位置0~1> <速度0~1>
```

例如：

```bash
./build/hand_finger_test left 1 0.5 1
```

表示将左手无名指设置到位置 `0.5`，速度为 `1`。

### 官方示例

```bash
./build/hand_example
```

该程序会交替控制左右手执行官方示例动作。
