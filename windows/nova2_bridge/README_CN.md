# Nova2 Windows Reader 准备与使用说明

这个目录用于 Windows 主机读取 SenseGlove Nova 2，并把数据通过 UDP 发到 Ubuntu/ROS2 电脑。

推荐路线：

```text
Windows 主机
  -> SenseCom
  -> SenseGlove Windows SDK
  -> nova2_windows_udp_reader
  -> UDP JSON

Ubuntu 22.04 ROS2 主机
  -> nova2_udp_bridge
  -> /manus_glove_0 / /manus_glove_1
  -> manus_revo2_retarget
  -> Revo2
```

这样 Ubuntu 端不需要直接链接 SenseGlove Linux `.so`，可以绕开 `GLIBC` / C++ ABI 不兼容问题。

## 目录内容

```text
nova2_windows_bridge/
  installers/
    download.md
  sdk/
    README.md
    include/SenseGlove/...
    lib/win64/msvc143/release/
      sgcore.dll
      sgcore.lib
      sgconnect.dll
      sgconnect.lib
  reader/
    CMakeLists.txt
    src/nova2_windows_udp_reader.cpp
    run_reader_example.ps1
  UDP_PROTOCOL_CN.md
  config/
    udp_schema_example.json
```

## 1. Windows 端安装 SenseCom

按照 `installers/download.md` 中的官方发布地址下载并在 Windows 主机上安装 SenseCom。

安装完成后启动 SenseCom，连接 Nova 2。确认 SenseCom 界面里能看到 Nova 2，并且能区分左手/右手。

如果使用官方便携版本，可能还需要手动安装 Visual C++ Runtime。

## 2. 准备 Windows 编译工具

推荐：

```text
Windows 10/11
Visual Studio 2022
CMake 3.20+
```

安装 Visual Studio 时勾选：

```text
Desktop development with C++
MSVC v143
Windows SDK
CMake tools for Windows
```

## 3. 编译 reader

打开 **x64 Native Tools Command Prompt for VS 2022**，或者打开 PowerShell 后先加载 Visual Studio 的 C++ 构建环境。

reader 的图形界面使用 **Qt 6 Widgets**，以自适应布局替代固定像素坐标，避免高 DPI、窗口缩放或字体变化造成控件重叠。

为保持 GitHub 仓库轻量，Qt 二进制不提交到仓库（`third_party\Qt` 已被 `.gitignore` 排除）。请在一台可联网的 Windows 电脑下载 Qt 6.8.2 `msvc2022_64`，再把下载结果复制到离线编译机。下载工具可使用 [aqtinstall](https://pypi.org/project/aqtinstall/)；它从 Qt 发布站下载预编译包。没有 Python 时，也可从 [aqtinstall Releases](https://github.com/miurahr/aqtinstall/releases/latest) 下载 `aqt.exe`。

在有网 Windows 电脑的 PowerShell 中执行：

```powershell
py -m pip install --upgrade aqtinstall
py -m aqt install-qt windows desktop 6.8.2 win64_msvc2022_64 -O D:\nova2_qt_cache
```

下载完成后，将以下目录完整复制到离线仓库中：

```text
来源：D:\nova2_qt_cache\6.8.2\msvc2022_64
目标：nova2_windows_bridge\third_party\Qt\6.8.2\msvc2022_64
```

离线编译 Windows 上仍需预先安装 Visual Studio 2022 的 Desktop development with C++ 工作负载、CMake 和 Visual C++ Runtime；这些是 Microsoft 工具链，不由 Qt 目录提供。Qt 下载和复制完成后，以下构建步骤不需要网络。

进入 reader 目录：

```powershell
cd path\to\Nova2Dex\windows\nova2_bridge\reader
```

以下命令完全离线。先从当前 `reader` 目录计算已复制 Qt kit 的绝对路径，再配置：

```powershell
$bridgeRoot = Split-Path -Parent (Get-Location)
$qtRoot = Join-Path $bridgeRoot "third_party\Qt\6.8.2\msvc2022_64"
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 "-DCMAKE_PREFIX_PATH=$qtRoot"
```

编译：

```powershell
cmake --build build --config Release
```

如果修改过 CMake、SDK 路径或遇到旧缓存问题，可以先清理构建目录后重新配置：

```powershell
Remove-Item -Recurse -Force build
$bridgeRoot = Split-Path -Parent (Get-Location)
$qtRoot = Join-Path $bridgeRoot "third_party\Qt\6.8.2\msvc2022_64"
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 "-DCMAKE_PREFIX_PATH=$qtRoot"
cmake --build build --config Release
```


编译完成后，可执行文件大致在：

```text
reader/build/Release/nova2_windows_udp_reader.exe
```

编译出的程序是 Windows 图形界面程序，采用三个页面：`Main` 用于配置和发送，`Data Visual` 专门展示实际发送的最新 JSON 帧，`Format` 展示 UDP v3 协议字段定义。启动后可以填写 Ubuntu IP、UDP 端口，选择左手/右手/双手，并查看 SenseCom、左右手电量及充电状态。

构建脚本会把 `sgcore.dll` 和 `sgconnect.dll` 复制到 exe 同目录。首次部署到没有 Qt 的电脑时，需要运行已复制 Qt kit 中的部署工具：

```powershell
& "$qtRoot\bin\windeployqt.exe" .\build\Release\nova2_windows_udp_reader.exe
```

这会把 Qt 运行所需的 DLL 和 `platforms` 插件复制到 exe 同目录。若目标电脑只运行已编译的程序，不需要 Qt kit、CMake 或网络；只需复制部署后的 exe 目录和 Visual C++ Runtime。

### Qt DLL 缺失或首次启动失败

如果编译成功、启动时却提示缺少 `Qt6Widgets.dll`、`Qt6Core.dll` 或 `Qt6Gui.dll`，原因是 CMake 已完成链接，但 Qt 运行时 DLL 尚未复制到 exe 目录。回到 `reader` 目录，先按前面的步骤设置 `$qtRoot`，然后执行：

```powershell
& "$qtRoot\bin\windeployqt.exe" .\build\Release\nova2_windows_udp_reader.exe
.\build\Release\nova2_windows_udp_reader.exe
```

`windeployqt` 会同时复制 Qt DLL 和 `platforms\qwindows.dll`。若之后提示缺少 `VCRUNTIME140_1.dll`，请离线安装与 Visual Studio 2022 匹配的 Microsoft Visual C++ Redistributable。

如果程序启动时报缺少 `sgcore.dll` 或 `sgconnect.dll`，确认这两个 DLL 位于 exe 同目录，并安装 Visual C++ Runtime。

## 4. 查 Ubuntu 电脑 IP

在 Ubuntu 电脑上执行：

```bash
ip addr
```

找到和 Windows 同一局域网的 IP，例如：

```text
192.168.1.20
```

后面 Windows reader 就往这个 IP 发 UDP。

## 5. 启动 Windows reader

确认 SenseCom 已安装。程序会在点击“开始发送”时自动检测 SenseCom；如果 SenseCom 未运行，会尝试通过 SDK 启动。

直接双击 GUI 程序：

```text
reader/build/Release/nova2_windows_udp_reader.exe
```

也可以在 PowerShell 中启动 GUI：

```powershell
cd path\to\Nova2Dex\windows\nova2_bridge\reader
.\build\Release\nova2_windows_udp_reader.exe
```

### GUI 控件说明

```text
Ubuntu IP：   目标 Ubuntu 主机的 IPv4 地址，默认 127.0.0.1
UDP 端口：    目标 UDP 端口，默认 15020
发送手型：    左手、右手或双手，默认双手
Device status：SenseCom 检测结果、左右手连接状态、电量与充电状态；检测到时状态框为绿色，未检测到时为灰色
开始发送：    按当前设置启动 60 Hz UDP JSON 发送；发送中显示绿色状态和包计数
结束发送：    停止 UDP 发送
Data Visual：使用 Left hand / Right hand 按钮切换；以可滚动的平面表完整展示对应手数据，关节数组按“一根手指一行”显示，每个关节/节点使用一个向量单元格，并在前面显示维度、顺序和单位说明
Format：      可滚动字段表，逐项说明 UDP v3 的字段路径、数据类型/维度、取值顺序、定义和 SenseGlove SDK 来源
```

IP 或端口输入框留空时，会分别回退到 `127.0.0.1` 和 `15020`。

三种手型对应的 UDP 内容：

```text
左手：hands 中只包含 side=left
右手：hands 中只包含 side=right
双手：hands 中同时包含 side=left 和 side=right
```

### reader 代码结构

```text
src/nova2_windows_udp_reader.cpp  SenseGlove SDK 读取、UDP 发送和状态查询
src/nova2_reader_backend.hpp      UI 使用的发送与状态接口
src/nova2_reader_window.cpp       Qt 页面、导航和显示逻辑
```

这样 UI 不直接调用 SenseGlove SDK；数据读取、UDP 发送和界面展示分别维护。

## 6. UDP JSON 数据格式

reader 每帧发送一条 JSON，大致格式：

```json
{
  "source": "senseglove_nova2",
  "version": 3,
  "timestamp_ms": 123456789,
  "hands": [
    {
      "side": "right",
      "glove_id": 1,
      "connected": true,
      "is_right": true,
      "sensor_data_valid": true,
      "sensor_channels": {
        "thumb_flexion": 0.0,
        "index_flexion_proximal": 0.0,
        "index_flexion_distal": 0.0,
        "middle_flexion": 0.0,
        "ring_flexion": 0.0,
        "thumb_abduction": 0.0
      },
      "imu_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
      "imu_valid": true,
      "battery_level": 0.85,
      "battery_valid": true,
      "is_charging": false,
      "hand_angles_rad": [
        [[0, 0.2, 0.01], [0, 0.1, 0], [0, 0.05, 0]]
      ],
      "joint_positions_mm": [
        [[0, 20, 0], [0, 40, 0], [0, 60, 0], [0, 80, 0]]
      ],
      "joint_rotations_xyzw": [
        [[0, 0, 0, 1]]
      ]
    }
  ]
}
```

完整示例见：

```text
config/udp_schema_example.json
```

字段、数组维度、单位和元素顺序见：

```text
UDP_PROTOCOL_CN.md
```

注意：Nova 2 的六路物理传感器不包含独立小拇指通道；小拇指屈曲由无名指通道关联得到。`sensor_channels` 是 reader 通过 SDK 按名称读取的协议字段，不代表 SDK 原始报文索引。具体字段见 `UDP_PROTOCOL_CN.md`。

## 7. Ubuntu 端下一步

Windows reader 发出 UDP 后，Ubuntu 端需要一个 `nova2_udp_bridge` 来接收 JSON 并发布：

```text
/manus_glove_0
/manus_glove_1
```

当前目录先准备 Windows 端。Ubuntu 端 bridge 会复用现有 `nova2_glove_driver` 的转换约定，但不再链接 SenseGlove SDK。

## 8. 网络注意事项

Windows 和 Ubuntu 必须在同一网络下。

如果 Ubuntu 收不到 UDP：

```bash
sudo ufw status
```

可以临时允许端口：

```bash
sudo ufw allow 15020/udp
```

也可以先在 Ubuntu 上抓包确认：

```bash
sudo tcpdump -ni any udp port 15020
```

## 9. 排查

### SenseCom 看不到手套

先不要运行 reader。只排查 SenseCom：

```text
1. Nova2 是否开机
2. 是否完成配对
3. USB dongle / 蓝牙是否正常
4. SenseCom 是否能显示设备
```

### reader 提示 SenseCom inactive

查看 GUI 中的 SenseCom 状态。如果仍未检测到，可以先手动打开 SenseCom，并确认 Nova 2 已配对、已开机且左右手标识正确。

点击“开始发送”时程序也会尝试自动启动 SenseCom。

### reader 有发送但 Ubuntu 没收到

检查：

```text
1. GUI 中 IP 是否是 Ubuntu IP
2. GUI 中 UDP 端口是否与 Ubuntu bridge 一致
3. Windows 和 Ubuntu 是否同一网段
4. Ubuntu 防火墙是否允许 UDP 端口
5. Windows 防火墙是否允许 reader 出站
```

### 数据方向不对

食指影响其他手指的 SDK 跟踪开关默认关闭。reader 在 `HandLayer` 实际使用的
Nova2 实例上设置开关，读取姿态前后校验，并将实际读回值放在每只手的
`index_influence_others` 字段中；Data Visual 页面应显示 `false`。

连接手套并启动 SenseCom 后，可运行硬件自检（不发送 UDP 或触觉命令）：

```powershell
$check = Start-Process .\build\Release\nova2_windows_udp_reader.exe -ArgumentList '--self-check' -Wait -PassThru -RedirectStandardOutput self-check.log -RedirectStandardError self-check.err
Get-Content self-check.log, self-check.err
$check.ExitCode
```

自检临时开启当前实例的开关，再调用实际数据打包路径关闭它，并重新获取
`HandLayer` 实例确认读回为关闭，最后恢复原值。退出码 `0` 表示通过，`1`
表示失败，`2` 表示没有连接 Nova2；它不替代单指动作的实际扰动检查。

Windows reader 读取 SGCore 的 HandPose 和 Nova2 设备状态，封装为 UDP JSON；它不在 Windows 端做 Revo2 映射调参。方向、零点、比例应在 Ubuntu 端 YAML 中调。

## 10. 版本说明

本目录当前打包的是：

```text
SenseCom 1.8.4
Windows SDK win64 msvc143 release
```

如果官方提供更新版本，可以替换：

```text
sdk/include/SenseGlove/
sdk/lib/win64/msvc143/release/*.dll
sdk/lib/win64/msvc143/release/*.lib
installers/
```

替换后重新编译 reader。
