# GELLO 机械臂遥操作配置与速度读取

适用于本目录的 `run_gello.py`、`gello/` 和 `gello_config.json`，以及本机另一个 SDK 中的 Panthera 实机入口。核对日期：2026-10-30。

本目录已经实现 GELLO → MuJoCo Panthera 双臂的位置遥操作。角度来自飞特 SMS/STS 风格协议的位置寄存器，经过跨圈展开、归零和两层方向映射后生成目标。当前 GELLO 读取接口已增加舵机速度寄存器反馈，调参窗口可显示六关节输入/映射角速度及夹爪舵机轴速度。Panthera 实机 SDK 则已有机械臂实际速度反馈接口。

## 1 选择运行入口

| 用途 | 入口 | 配置来源 | 是否驱动实机 |
| --- | --- | --- | --- |
| 检查 GELLO 连接 | 本目录 `检查Gello连接.py` | `gello_config.json` | 只读主手位置和速度 |
| 单独调试主手和映射 | 本目录 `gello_tuner.py` | `gello_config.json` | 只读主手位置和速度 |
| 控制 MuJoCo 双臂 | 本目录 `run_gello.py` | `gello_config.json` | 控制仿真 |
| 控制 Panthera 实机单臂 | `../../Panthera-HT_SDK/panthera_python/scripts/7_gello_teleop.py` | 命令行、`Follower.yaml`、电机 YAML | 会驱动实机 |

本目录程序没有机械臂 CAN/SDK 输出接口，修改 JSON 不能将仿真切换成实机。SDK 中的实机脚本是独立实现，不加载本目录的 JSON。旁边的 Ragtel 又有自己的 `teleop_gui.ini`，不能直接互换配置。

## 2 串口和节点配置

进入当前项目：

```bash
cd ~/workspace/遥操资料_20261030_完整版/MJC-Dishwasher_Gello遥操
ls -l /dev/serial/by-id/
```

将 `gello_config.json` 中的 `serial.port` 改成实际 GELLO 设备路径。稳定的 `/dev/serial/by-id/...` 路径便于拔插后重连；当前保存的是 `/dev/ttyACM0`。机械臂通信板和 GELLO 串口应分别识别。

同一条 GELLO 总线同一时刻只运行一个读取程序。`run_gello.py` 自带调参窗口，无需再启动 `gello_tuner.py`。接线、电源和适配器类型须匹配实际舵机：STS 与 SMS 的物理接口不同，不能根据 `/dev/ttyACM*` 名称判断 TTL 或 RS485。[飞特 STS 说明](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/)、[SMS 说明](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sms/)

当前串口配置如下，数值来自随包 JSON：

| 字段 | 当前值 | 含义 |
| --- | --- | --- |
| `serial.port` | `/dev/ttyACM0` | GELLO 串口 |
| `serial.baud` | `1000000` | 1 Mbps，需与设备一致；代码使用 8N1 |
| `serial.group_mask` | `3` | `1` 只扫描 ID 1–7；`2` 只扫描 ID 11–17；`3` 扫描两组 |
| `serial.timeout_ms` | `5.0` | 每个节点读取的超时时间 |
| `serial.scan_interval_ms` | `0.0` | 每轮扫描不额外等待，不代表采样周期为零 |
| `serial.read_velocity` | `true` | 同帧读取位置与速度；关闭后恢复两字节位置读取 |
| `serial.velocity_rpm_per_unit` | `0.732` | 速度寄存器每单位对应 RPM，须与实物相位配置一致 |
| `serial.max_delta_ticks` | `100` | 异常跳变触发复读的阈值，约 8.79°；不是速度上限 |
| `serial.auto_reconnect` | `true` | 可恢复的串口故障自动重连 |
| `serial.link_loss_ms` | `500.0` | 所有已启用节点均无合法响应的掉线阈值 |
| `serial.reconnect_initial_ms` | `250.0` | 首次重连等待 |
| `serial.reconnect_max_ms` | `2000.0` | 指数退避的最大等待 |
| `swap_arms` | `true` | 当前开启左右目标臂交换 |

每组必须有六个关节舵机和一个夹爪舵机，顺序在 `gello/reader.py` 中固定：

| 主手组 | 六关节 ID | 夹爪 ID | 使用的 JSON 参数 | 当前目标臂 |
| --- | --- | --- | --- | --- |
| group 0 | 1、2、3、4、5、6 | 7 | `left` | MuJoCo 右臂 |
| group 1 | 11、12、13、14、15、16 | 17 | `right` | MuJoCo 左臂 |

`swap_arms=false` 时，group 0 控制左臂，group 1 控制右臂。交换目标臂不会交换各组使用的 `left`/`right` 参数。只接一组时设置对应的 `group_mask`，避免轮询不存在的节点。

## 3 仿真启动和标定

先使用随包虚拟环境执行不连接硬件的检查：

```bash
./.venv/bin/python gello_tuner.py --check-config
./.venv/bin/python run_gello.py --smoke-test
```

如需在新机器重建环境，依赖在 `requirements.txt`；GUI 还需要 Python 的 Tk 支持和可用的 OpenGL。不要照搬旧 README 中原作者的 Python 绝对路径。

串口配置完成后，可单独做位置读取检查：

```bash
./.venv/bin/python 检查Gello连接.py
```

该脚本读取约 3 秒后退出。确认启用组中的七个节点均有数据，完整扫描数增长。串口能打开只代表设备文件可访问，不代表舵机已响应。

启动遥操：

```bash
./运行_Gello遥操.sh
# 等效入口，可指定自己的配置文件：
# ./.venv/bin/python run_gello.py --config gello_config.json --control-hz 60
```

1. 将 GELLO 保持在希望对应仿真初始姿态的位置，再在 Tk 窗口点击“连接”。每组第一份七节点完整数据自动成为该组零位；当前仿真六关节初始角均为 `0 rad`。
2. 逐轴小幅转动主手，检查实际受控侧、关节顺序和方向。方向相反时修改对应 `axes` 项的 `sign`；缩放用 `scale`；姿态偏移用 `offset_rad`，单位是弧度。
3. 分别对夹爪点击“开始/结束标定”，完整开合一次，再结束标定。确认开合方向，必要时切换 `inverted`。
4. 点击“应用到 MuJoCo”使映射生效；“保存参数”会同时应用并写回 JSON。串口参数需要断开并重新连接才生效。
5. 关闭任意窗口结束程序。也可先用 `gello_tuner.py` 完成主手调试，退出后再启动仿真。

手动归零会改变当前参考姿态。零位存在 reader 内存中，未写入 JSON；同一个 reader 自动重连时保留零位，手动断开后新建 reader 会重新取零。长时间掉线期间若主手转过半圈以上，最短路径展开可能无法恢复真实圈数，应重新对齐、归零。

夹爪输出范围 `0～0.04 m` 表示**单侧手指位移**，两指总开口可达约 `0.08 m`。界面中的 `0～0.6 rad` 是标定行程的显示量，不是机械臂夹爪电机角度。当前标定值为：

| 参数组 | `min_raw` | `max_raw` | `inverted` | 输出范围 |
| --- | --- | --- | --- | --- |
| `left`，group 0 | 95 | 823 | `true` | 0～0.04 m |
| `right`，group 1 | 1257 | 2045 | `false` | 0～0.04 m |

虽然字段叫 `raw`，夹爪标定实际使用连续展开 tick，允许小于 0 或大于 4095。换主手或重新装配后应重新标定，不能直接沿用这两个范围。

## 4 代码如何读出角度

读取链路如下：

```text
gello/protocol.py：构造位置+速度读取请求、校验和解码响应
  → gello/reader.py：逐 ID 轮询、跨圈展开、归零、固定方向换算
  → GroupSnapshot.arm_radians：六个相对零位的关节角，rad
  → gello/mapping.py：应用 JSON 的方向、缩放、偏移
  → run_gello.py：选择左右目标臂
  → DishwasherSim.set_arm_angles / set_gripper：裁剪后写仿真目标
```

### 4.1 报文和原始位置

兼容接口 `build_position_request()` 读取地址 `56 / 0x38` 的两字节当前位置。ID 1 的实际请求是：

```text
FF FF 01 04 02 38 02 BE
帧头  ID 长度 读 地址 字节数 校验
```

响应经过帧头、长度、校验和和 ID 检查。位置低字节在前，bit 15 按符号位解码，再取 `% 4096` 得到单圈 `0～4095` tick。这里是符号加幅值编码，不能直接按有符号 int16 补码解释。

相邻 tick 差值按最短方向展开。例如 `4090 → 5` 为正向 `11 tick`，不是负向 `4085 tick`。`100 tick` 阈值触发复读；两次读数一致时，大于阈值的变化仍可能被接受，因此它不能代替机械臂限速。

### 4.2 角度和两层方向

对第 i 个旋转关节，代码使用：

```text
相对输入角 q_in = SERVO_SIGNS[i] × SERVO_GEARS[i]
                   × (unwrapped_ticks[i] − zero_ticks[i]) × 2π / 4096

目标角 q_target = q_neutral[i]
                  + axes[i].sign × axes[i].scale × q_in
                  + axes[i].offset_rad
```

六关节固定的 `SERVO_SIGNS = [-1, -1, +1, -1, +1, -1]`，`SERVO_GEARS` 均为 1；它们在 JSON 映射之前已经生效。每 tick 为 `0.00153398 rad`，即 `0.087890625°`。不要再对 `arm_radians` 乘一遍固定符号。

当前 JSON 的 `scale` 全为 1，`offset_rad` 全为 0。两层符号合并后为：

| 关节 | reader 固定符号 | `left.axes.sign` | group 0 合成符号 | `right.axes.sign` | group 1 合成符号 |
| --- | --- | --- | --- | --- | --- |
| J1 | -1 | -1 | +1 | +1 | -1 |
| J2 | -1 | -1 | +1 | -1 | +1 |
| J3 | +1 | -1 | -1 | -1 | -1 |
| J4 | -1 | +1 | -1 | -1 | +1 |
| J5 | +1 | -1 | -1 | -1 | -1 |
| J6 | -1 | +1 | -1 | +1 | -1 |

例如 group 0 的 J1 比零位增加 1024 tick，输入角是 `−π/2`，经 `left.axes[0].sign=-1` 后，仿真目标增加 `+π/2`。

夹爪使用展开位置在标定范围内的比例，再应用 `inverted` 和米制输出范围；不套用六关节公式。reader 中第七轴的 gear 常量为 4，也不应直接拿来换算当前标定夹爪的线速度。

### 4.3 完整性和目标限位

一组七个节点当轮都成功才会标记 `data_valid=True`。单侧缺节点时，该侧保持之前目标，完整的另一侧仍可更新。舵机 status 非零会计入告警，但当前 reader 仍接受帧和位置合法的响应；`data_valid` 不表示硬件无故障。

MuJoCo 会按 actuator 范围裁剪目标，例如当前 J2 为 `0～3.2 rad`，J3 为 `0～4 rad`。如果原始值和映射角变化而模型不动，先检查目标是否落在限位之外。

`--control-hz 60` 是仿真控制频率，不是主手完整扫描频率。主手为逐节点轮询，应看调参窗口的扫描 Hz、p95/p99 扫描时间和节点超时统计。

## 5 使用本机 Panthera 实机脚本

实机入口位于本资料包之外的 [Panthera SDK](../../Panthera-HT_SDK/panthera_python/README.md)。拷贝资料包到别的机器时，需要另外部署该 SDK 和机械臂通信环境。

### 5.1 运行环境和从臂配置

使用安装好 `hightorque_robot`、`Panthera_lib` 依赖和 `feetech-servo-sdk` 的 Python 环境。本目录 `.venv` 的 MuJoCo 依赖不能替代机械臂 SDK 环境。依赖安装按 SDK README 执行。

实机配置文件：

| 文件 | 需要核对的内容 |
| --- | --- |
| `panthera_python/robot_param/Follower.yaml` | `gripper_installed`、关节限位、夹爪限位、`param_file`、URDF 路径 |
| `panthera_python/robot_param/motor_param/6dof_Panthera_params_follower.yaml` | 通信板串口配置、电机数量、ID、关节名称与实际接线对应关系 |

当前电机 YAML 的 `Serial_Type` 为 `/dev/ttyACM`，`Seial_baudrate` 为 `4000000`，这是机械臂侧配置；GELLO 仍使用 1 Mbps。保留代码要求的字段拼写。当前 `motor_timeout_ms=0` 表示禁用电机通信超时，不能将主机 GELLO 0.5 秒超时理解成独立的电机看门狗。

### 5.2 先检查主手再启动从臂

在 SDK 环境中，从 `scripts` 目录运行；以下 `/dev/ttyACM0` 需替换为实测 GELLO 串口：

```bash
cd ~/workspace/Panthera-HT_SDK/panthera_python/scripts
python 7_gello_teleop.py --gello-port /dev/ttyACM0 --dry-run
```

`--dry-run` 只连接 GELLO，不创建 Panthera 从臂。等七节点读取正常，将主手摆到参考姿态，按 Enter 捕获零位，逐轴检查输出的相对角度。该脚本固定读取 ID 1–7，不支持通过 `group_mask` 选择 11–17。

正式运行前须确认回零路径无干涉：**下面的实机入口在 GELLO 预检成功、机械臂初始化后，会倒计时 3 秒，自动以 0.3 rad/s 将六关节回到零位，并默认闭合夹爪；完成后才等待 Enter 标定。**

```bash
python 7_gello_teleop.py \
  --gello-port /dev/ttyACM0 \
  --follower-config ../robot_param/Follower.yaml \
  --joint-signs -1 -1 -1 -1 -1 -1 \
  --joint-scales 1 1 1 1 1 1 \
  --control-rate 100 \
  --gello-timeout 0.5
```

这里的方向参数是脚本当前默认值，须以 dry-run 和逐轴核对结果为准。该实机脚本直接对展开 tick 应用这一层符号，没有仿真 reader 的固定 `SERVO_SIGNS` 层，因此不能直接复制 JSON 的 `axes.sign`。若需要复现仿真的输入方向，应先计算两层符号的乘积，再核对实机安装方向。

启动标定后的映射为：

```text
q_target = 从臂标定时实测关节角
           + joint_signs × joint_scales × (当前展开tick − 主手零位tick) × 2π/4096
```

实机脚本的主要参数和行为：

| 项目 | 当前实现 |
| --- | --- |
| 关节 MIT Kp / Kd | `[20,40,50,30,15,10]` / `[2,2.5,2.5,2,1.5,1]`，属于现有默认值，非通用整定结果 |
| 位置目标 | 按 `Follower.yaml` 关节限位裁剪 |
| 速度目标 | 固定为六个 0，不使用 GELLO 速度前馈 |
| 速度与加速度限制 | 遥操循环没有应用 YAML 中的速度/加速度限制，也没有低通或软件斜率限制 |
| 夹爪 | `--gripper-sign=-1`，`--gripper-tick-span=700`，映射到从臂夹爪关节角范围，不是米 |
| 夹爪 MIT Kp / Kd | 实际常量为 `5.0 / 0.2`；`--help` 的文字仍写 `4.0 / 0.4`，以常量为准 |
| `--no-gripper` | 不控制从臂夹爪，但 GELLO 仍要求 ID 7 响应；无实机夹爪还应正确设置 YAML 的 `gripper_installed` |
| 丢帧 | 短暂丢帧继续发旧目标，超过 `--gello-timeout` 抛错退出 |
| 退出 | 尝试发送三帧当前位置保持命令后关闭 GELLO；不能据此保证断电或通信失败后的持续保持 |

不要用 `--control-rate` 或 tick 跳变阈值代替实机关节速度限制。SDK 实机脚本文件头的旧示例仍写 `5_gello_follower_teleop.py`，实际文件名是 `7_gello_teleop.py`。

## 6 速度能否读取

### 6.1 GELLO 舵机速度反馈

**已实现 GELLO 主手速度读取，默认开启。** `run_gello.py` 和 `gello_tuner.py` 共用采集线程和调参窗口；`检查Gello连接.py` 也会打印原始速度和 rad/s 速度。读取功能不写舵机寄存器，不改变扭矩或运行模式，尚未接入速度前馈。

`build_position_velocity_request()` 从地址 56 连续读取 4 字节，以 ID 1 为例：

```text
请求：FF FF 01 04 02 38 04 BC
响应：FF FF ID 06 STATUS POS_L POS_H SPEED_L SPEED_H CHECKSUM
```

`parse_position_velocity_response()` 校验帧头、长度、校验和及可选 ID，位置和速度均按低字节在前、bit 15 符号加幅值解码。速度保留正负号，不做 `%4096`。原有两字节位置协议函数保留；在界面取消“读取主手速度”后重新连接，即恢复位置读取，不自动掩盖不支持四字节响应的设备问题。

速度换算：

```text
舵机轴速度 = raw_velocity × velocity_rpm_per_unit × 2π/60
六关节输入速度 = 舵机轴速度 × SERVO_SIGNS[i] × SERVO_GEARS[i]
六关节映射速度 = 输入速度 × axes[i].sign × axes[i].scale
```

归零、固定偏移和仿真初始姿态不参与速度换算。映射速度仅用于显示，是限位裁剪之前的主手映射值，不是 MuJoCo 实际速度。夹爪显示的是原始舵机轴角速度，不乘第七轴 gear、不当作手指线速度。

飞特官方 STS/SMS 表将当前速度放在 `58 / 0x3A`，两字节，bit 15 为方向位。相位 BIT2 为 0 时单位 `0.732 RPM`，为 1 时 `0.0146 RPM`。当前程序默认前者，没有自动读取或修改相位；请按实物型号、固件和设置选择界面的“速度单位 RPM/原始值”，修改后重新连接。此设置对总线上的全部节点生效，节点速度单位需一致。[STS 内存表](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/)、[SMS 内存表](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sms/)

快照新增字段：

| 字段 | 含义 |
| --- | --- |
| `GroupSnapshot.raw_velocities` | 七个节点的有符号速度寄存器值 |
| `arm_velocities_rad_s` | 六关节经过 reader 固定方向和 gear 换算的 rad/s |
| `gripper_servo_velocity_rad_s` | 夹爪舵机轴速度 rad/s |
| `velocity_valid` | 当轮七个节点都有合法且无状态告警的速度 |
| `servo_captured_at` | 七个节点响应接收时的单调时钟时间，不是硬件采样时刻 |

逐节点缺失值为 `None`，界面显示 `—`。超时、错误帧、被拒绝的位置跳变不会沿用旧速度；非零舵机状态继续沿用原来的位置处理，但速度标为无效；断开、重连等待或采集失败会使旧快照速度失效。`data_valid` 仍表示位置完整性，不能替代 `velocity_valid`。使用快照时还需检查连接状态和时间戳的新鲜度。

现有 reader 的使用示例（不另开串口）：

```python
snapshot = reader.latest_snapshot()
if snapshot is not None and reader.is_connected:
    group = snapshot.group(0)
    if group is not None and group.velocity_valid:
        print(group.arm_velocities_rad_s)
        mapped = tuple(axis.apply_velocity(speed) for axis, speed in
                       zip(config.left.axes, group.arm_velocities_rad_s))
```

尚未连接实物验证卸力拖动时的反馈、单位、低速分辨率和更新率。此处实现的是寄存器反馈，未使用位置差分估计。

### 6.2 用角度差分估计主手速度

无需增加总线报文，可使用新到达的连续角度快照：

```text
ω[i] = (q_now[i] − q_prev[i]) / (t_now − t_prev)     单位 rad/s
```

使用 `arm_radians` 或展开 tick，避免对 `raw_positions` 直接差分造成跨圈尖峰。`captured_at` 使用单调时钟，`sequence` 用来排除重复快照；不能把 GUI 刷新周期或 1/60 秒当成实际采样间隔。

当前 `captured_at` 是整轮扫描结束时间，14 个舵机逐个读取，并非同时采样。用它可以做粗略速度估计；新增的 `servo_captured_at` 提供每节点响应接收时间，可用于更细粒度的时间对齐，但不等于硬件采样时刻。首帧、归零、重连、长时间丢帧后应重置差分历史，参数修改后也应重置相关滤波状态。量化噪声可用低通或滑窗斜率减小，但会增加延迟。

由角度差分得到的是估计速度；寄存器速度是舵机上报值。将两者并行记录，可以检查单位、方向和延迟。若进一步用于实机 MIT 速度前馈，还需单独设计速度/加速度限幅和失效处理，不能把新增字段直接替换当前的零速度目标。

### 6.3 Panthera 实际速度与仿真速度

若要的是**从臂实际关节速度**，本机 `Panthera_lib/Panthera.py` 已实现 `robot.get_current_vel()`，逐关节读取电机状态的 `velocity`，SDK 文档单位为 rad/s。夹爪有 `get_current_vel_gripper()`。这些方法读取 SDK 当前状态缓存，应按通信流程刷新并确认反馈新鲜度；不要为读取速度另外创建一个抢占同一通信板的控制实例。参考 [实机状态示例](../../Panthera-HT_SDK/panthera_python/scripts/0_robot_get_state.py)。

若要的是**MuJoCo 实际关节速度**，可在仿真进程内按关节的 DOF 地址读取：

```python
import mujoco

# sim 是已有 DishwasherSim；下面读取仿真左臂 J1。
joint_id = mujoco.mj_name2id(
    sim.model, mujoco.mjtObj.mjOBJ_JOINT, "left/joint1"
)
if joint_id < 0:
    raise KeyError("left/joint1")
dof_address = sim.model.jnt_dofadr[joint_id]
velocity_rad_s = float(sim.data.qvel[dof_address])
```

`qvel` 使用 `jnt_dofadr`，不能用读取角度时的 `jnt_qposadr`。这是物理仿真的实际速度，与 GELLO 主手速度、目标角度变化速度分别属于不同信号。

另有 `encoder/` 的 AS5047P RS485 协议分支：源码说明其响应第 4～6 字节（从 0 开始编号）带角速度，但当前解析器不导出该字段。它不属于 `gello/` 的舵机协议；具体编码和单位需查对应固件，不能套用地址 58 的规则。

## 7 常见问题和检查结果

| 现象 | 检查位置 |
| --- | --- |
| 串口打开但无角度 | 实际串口、供电、波特率、适配器类型、总线是否被另一个程序占用 |
| 单侧一直无效 | `group_mask` 和该组七个节点，尤其夹爪 ID 7/17 是否响应 |
| 转左手却动右臂 | 当前 `swap_arms=true`；参数仍跟随 ID 组 |
| 方向与 JSON sign 直觉相反 | 检查 reader 固定 `SERVO_SIGNS` 和 JSON 的两层乘积 |
| 目标变化但仿真关节不动 | 检查 actuator 限位，尤其 J2/J3 的负向目标 |
| 仿真无法开窗 | Tk、显示环境和 OpenGL；仅需诊断软件渲染时可用 `--allow-software-rendering` |
| 重启后零位改变 | JSON 不保存关节零位，每个新 reader 首份完整数据重新归零 |
| 把仿真 JSON 用到实机没有效果 | 实机脚本读取自己的命令行和 YAML，不读取本目录 JSON |
| 速度一直为 0 | 先区分目前固定为 0 的 MIT 目标速度、主手速度反馈字段和真实反馈速度 |

本次已完成的无硬件验证：配置校验通过；GELLO → MuJoCo smoke test 通过，输出 `0 -> right, 1 -> left`；速度功能完成后以下 43 项测试全部通过，覆盖协议、跨圈、标定、重连、伪终端串口和仿真映射：

```bash
./.venv/bin/python -m unittest discover -s tests -p 'test_gello*.py'
```

实机脚本仅核对源码和 `--help`，没有启动回零或驱动机械臂；也未连接舵机验证速度寄存器。本次修改限定于本目录的 GELLO 主手速度采集、显示、配置和测试；未修改 Ragtel 或包外实机脚本。
