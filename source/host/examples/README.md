# Panthera-HT UDP 控制与标定示例

本组示例直接使用 `host/robot_sdk.py`，复用现有 Zephyr 固件。动力学在 PC 计算，电机执行 MIT 五参数。机械臂命令单位为 rad、rad/s、Nm、Nm/rad、Nm·s/rad。

## 入口

| 文件 | 功能 |
|---|---|
| `3_gravity-compensation.py` | 实际姿态 G(q)，Kp/Kd=0，自由拖动 |
| `4_impedance-control.py` | 电机端 PD＋PC 重力前馈；当前姿态保持或五次曲线到目标 |
| `5_gravity-calibration.py` | 多姿态静态采集、稳定性评估、等效重力辨识 |
| `6_friction-calibration.py` | 单关节、多速度、正反向匀速摩擦采集 |
| `7_static-friction-calibration.py` | 正反向缓慢增加额外力矩，检测起动阈值 |
| `8_gravity-friction-compensation.py` | G(q)＋库伦/黏滞摩擦前馈 |
| `9_identify-offline.py` | 重力、运动摩擦及静摩擦数据的离线分析 |
| `10_make-calibration-plan.py` | 离线生成 243 点或历史完整 285 点姿态文件 |
| `11_priority-pose-capture.py` | 三个指定姿态慢速、高增益补采，循环三轮 |
| `12_priority-gravity-fit.py` | 新姿态90%＋原数据10%，比较 link6 与全臂静态重力拟合 |
| `13_pooled-link6-fit.py` | 合并数据，每个合格采样点等权，仅拟合 link6，力矩只使用 J2–J6 |
| `14_random50-hand-com-capture.py` | 原285姿态随机抽50个，1/1/1/1秒，手总质量624±30g，仅标定link6 |
| `15_gravity-hand-sliders.py` | 已选重力＋60%摩擦，手模式3，六个Dear PyGui滑条及角度/电流回读 |
| `16_gravity-five-finger-grasp.py` | 五指抓握百分比版本，拇指旋转到100°，保留独立关节控制和角度/电流回读 |
| `17_gello-arm-compare.py` | COM5 GELLO双组角度/速度读取，与机械臂纯重力补偿状态并排比较 |

例15：先结束占用同一UDP端口的旧控制程序，再从host目录运行 ` .\examples\run_example.cmd 15_gravity-hand-sliders.py --execute`。不带`--execute`只打开离线窗口。臂目标发送/状态发布500Hz，手目标发送/状态发布50Hz，手角度及电流各查询50Hz；界面显示实测发送、接收及字段反馈频率。滑条初始化为当前反馈，六通道依次为拇指旋转、拇指弯曲、食指、中指、无名指、小指弯曲。手模式3，沿用500mA电流配置，可用`--hand-current-ma`修改。关闭窗口或Ctrl+C结束，退出臂保持逻辑沿用例3。`raw.csv`记录臂，`hand.csv`记录手目标、角度、电流及反馈年龄。Dear PyGui 2.3.1已安装；外部环境需安装`dearpygui>=2,<3`。界面按官方[渲染循环](https://dearpygui.readthedocs.io/en/latest/documentation/render-loop.html)与[手动回调](https://dearpygui.readthedocs.io/en/latest/documentation/item-callbacks.html)机制运行，设备控制独立于GUI线程。

所有入口可单独运行。3～8 未带 `--execute` 时只做模型预览，不打开 UDP。9 始终离线。按 Ctrl+C 结束。当前实现不增加位置、速度、力矩、变化率限幅，不修改原固件；会拒绝非有限值或缺失/过期反馈，避免用无效数据计算动力学。

## 运行环境

例17从host运行 `.\examples\run_example.cmd 17_gello-arm-compare.py --execute`：默认COM5、1000000波特、读取两组ID 1–7及11–17，窗口切换显示组。臂使用已选重力模型，500Hz发送/反馈；GELLO数据只显示和记录，不发送跟随目标，也不控制灵巧手。`--gello-only`只读串口，既不连接也不控制机械臂；无开关为离线界面。新实现 `gello_arm_compare.py` 和独立驱动 `gello_compare_driver/` 均在examples下，原更新包未改动。Windows串口超时默认30ms，可用`--serial-timeout-ms`修改；驱动使用高精度perf_counter计时。

GELLO位置4096ticks/圈；速度为地址58读出的符号位幅值原始速度，默认0.732RPM/单位，可用`--velocity-rpm-per-unit 0.0146`匹配另一种舵机相位设置。原示例读取方向[-1,-1,+1,-1,+1,-1]保留，不额外套用MuJoCo左右臂映射。窗口显示原始角度、相对参考变化量、两边速度、变化量差值、反馈年龄；Flip仅翻转显示和日志，不更改设备。两边摆好相同姿态后点Capture comparison reference重新记录比较基准，不写设备零点。初次收到两边完整数据会自动捕获初始参考。日志comparison.csv是界面采样快照，包含两组数据和采样年龄；arm/raw.csv记录臂控制反馈。GELLO扫描不是六轴同时采样，界面显示实测整总线扫描率。

例16另设五指抓握曲线，不改变例15二指曲线。从host运行 `.\examples\run_example.cmd 16_gravity-five-finger-grasp.py --execute`，取消顶部 `Individual finger control` 勾选，显示五指百分比滑条。0–20%：拇指旋转0→100°、其余弯曲角均0°；20–95%：拇指/食指/中指/无名指/小指弯曲分别0→13/53/56/55/52°；95–100%：五个弯曲角各再增加5°，终点18/58/61/60/57°。20%后拇指旋转保持100°。各段内部线性且交界连续，反向同曲线。补偿、控制频率、模式3和回读沿用例15。

例15窗口最上方 `Individual finger control` 默认打开，保留六通道独立控制。取消勾选切换为单个二指夹爪百分比滑条，首次为0%，中指/无名指/小指目标立即设为90°。0–20%：拇指旋转0→75°，拇指弯曲/食指弯曲均0°；20–95%：旋转保持75°，拇指弯曲0→18°、食指弯曲0→52°；95–100%：旋转保持75°，拇指弯曲18→20°、食指弯曲52→55°。各段内部线性，反向使用相同曲线，交界连续。切回独立模式保留当前目标；再次进入夹爪模式恢复上次百分比。角度、电流回读在两种模式下均显示。

本机已验证解释器：`C:\Users\KelvinLM\miniconda3\envs\L1-Hand-PantheraHT\python.exe`，NumPy 2.5.4、Pinocchio 4.1.0。直接从 IDE 启动也支持；`arm_runtime.py` 为 Windows Conda 补齐 native DLL 查找路径。

在本目录 PowerShell 中运行，例如：

```powershell
.\run_example.cmd 3_gravity-compensation.py
.\run_example.cmd 3_gravity-compensation.py --execute --seconds 15
```

也可激活已有 Conda 环境后 `python 文件名.py 参数`。其他电脑需安装 `requirements-arm.txt`；发行包名是 `pin`，Python 导入名是 `pinocchio`。

## 模型与零位

当前例 3 默认加载用户确认的 `profiles/gravity_selected.json`，来源为 `logs/random50_hand624_001/gravity_identified.json`：link6 总质量 **0.594 kg**，质心 **[60.55724, 4.66760, -12.10136] mm**，link1～link5 固定。该结果在总质量 624±30 g 约束下达到下限，质量不是称重结果；标定仅使用 J2～J6 力矩。

手替换整个旧 link6，不叠加旧 link6 质量。底层 `host/Assets/PantheraHT_v4_hand620g.urdf` 保留用于历史数据的模型身份兼容；文件中的 0.62918329 kg 是早期错误叠加 9.18329 g 的初值，例 3 的已选辨识参数会覆盖它。其他示例不会自动切换到例 3 的选定参数，需显式传入相应 `--identified`。转动惯量未做动态标定。

`q_model = sign * q_encoder + offset_rad`；重力力矩按同样 sign 转回电机坐标。默认 sign 全 1、offset 全 0 只是初始映射，实际零位和方向仍需核对。可编辑 `mapping.identity.json` 的副本并用 `--mapping 文件.json`。不写电机 EEPROM、不重置零点。

额外负载可用 `--payload-mass kg --payload-com x y z` 描述，质心单位 m，坐标系是 URDF 的 **joint6 坐标系**。已选辨识参数已经包含手的全部质量，不要重复添加；改变负载配置需与辨识文件身份匹配。底座倾斜时使用 `--gravity gx gy gz`，单位 m/s²，表达在模型基坐标系。

重力辨识文件通过 `--identified gravity_identified.json` 使用。模型会检查 URDF 哈希、映射、负载和重力方向的一致性；运行和辨识时应传同一套模型参数。

## 3：重力补偿

```powershell
.\run_example.cmd 3_gravity-compensation.py --execute --hz 500 --seconds 15
.\run_example.cmd 3_gravity-compensation.py --execute --identified logs\gravity01\gravity_identified.json --seconds 0
```

例 3 不指定 `--seconds` 时默认持续运行，按 Ctrl+C 退出；`--seconds 0` 也表示持续运行，正数表示运行指定秒数。`--gravity-scale` 默认 1。计算使用实际反馈 q，不使用目标角度。500 Hz 是请求频率，日志记录实际发送频率、周期平均值和最大周期。

## 4：阻抗控制

例 4 不指定 `--seconds` 时默认持续运行，按 Ctrl+C 退出；显式指定正数则按指定秒数运行。

```powershell
.\run_example.cmd 4_impedance-control.py --execute --seconds 15
.\run_example.cmd 4_impedance-control.py --execute --target-deg 0 45 45 0 0 0 --transition 5 --seconds 30
```

不指定目标时保持启动反馈姿态。目标角度以**电机编码器坐标**表示。`--transition` 是轨迹时长，不是限速器。默认 Kp=[4,10,10,2,2,1]、Kd=[0.5,0.8,0.8,0.2,0.2,0.1]，来自旧实验的初始增益，当前新装配尚未实机调好。可用 `--kp`、`--kd` 分别输入六个数。

电机计算 `Kp*(q_ref-q)+Kd*(dq_ref-dq)+tau_ff`；PC 只给重力/摩擦前馈，不叠加第二遍 PD。

## 5：重力标定

例 5 默认使用历史调大的增益：Kp=[24.48,61.2,61.2,12.24,12.24,12.24]，Kd=[1.2,1.92,1.92,0.48,0.48,0.48]，顺序 J1～J6。可通过 `--kp`/`--kd` 覆盖；增益改变后应新建数据目录。

```powershell
.\run_example.cmd 5_gravity-calibration.py --execute --output logs\gravity285_new
```

默认 `--plan-preset legacy285`，使用历史绝对角度的完整 285 点，J1 固定为 0°。顺序与 2026-10-29 记录相同：

| 点号 | 扫描 | 固定姿态（度） | 范围与点数 |
|---|---|---|---|
| 1～7 | J6 | J2=90,J3=180,J4=-90,J5=0 | -135～135，7 点 |
| 8～14 | J5 | J2=90,J3=180,J4=0,J6=0 | -90～90，7 点 |
| 15～21 | J4 | J2=90,J3=180,J5=0,J6=0 | -90～90，7 点 |
| 22～28 | J3 | J2=90,J4=J5=J6=0 | 200→45，7 点 |
| 29～35 | J2 | J3=135,J4=J5=J6=0 | 30～160，7 点 |
| 36～278 | J2～J6 组合 | 基准 [0,90,90,0,0,0] | 各取基准加 [0,+30,-30]，243 点 |
| 279～285 | J2 | J3=10,J4=30,J5=0,J6=90 | 5～70，7 点 |

此计划源自旧夹爪记录，当前新夹爪的碰撞情况尚未验证。恢复原先的启动姿态附近 243 点可显式指定 `--plan-preset grid243 --span-deg 15`。`--span-deg` 不改变 legacy285 固定范围。需要指定点位时使用优先级更高的 `--plan poses.json`，格式：

```json
{"poses_deg": [[0,45,45,0,0,0], [0,50,45,0,0,0]]}
```

上面仅演示格式；辨识至少需要 10 个稳定姿态，并需要足够的姿态覆盖。每点默认：`--transition 1` 秒五次插补 → `--settle 0.5` 秒等待 → `--sample 0.5` 秒采集 → `--post-hold 0.5` 秒保持，然后记录完成。共 2.5 秒/点，285 点理论 712.5 秒（11 分 52.5 秒），另加启动和写盘开销。以实际位置、实际力矩的均值辨识；最大关节角标准差超过 `--stability-deg 0.2` 的点保留原始记录但不参与拟合。缩短等待后未稳定的点仍会保留但不纳入拟合。

可先生成计划并检查，再交给例 5：

```powershell
.\run_example.cmd 10_make-calibration-plan.py --preset grid243 --center-deg 0 45 45 0 0 0 --span-deg 15 --output poses243.json
.\run_example.cmd 10_make-calibration-plan.py --preset legacy285 --output legacy285.json
```

legacy285 原样复现旧记录的五组 7 点扫描、243 点组合和 J2 追加 7 点，现已按用户要求作为例 5 默认计划。例 10 与例 5 共享同一份点位生成逻辑。

输出 `points.json`、`plan.json`、`raw.csv`；稳定点足够时自动生成 `gravity_identified.json`。当前默认 `--fit-scope link6`，只拟合 link6 的 m、mx、my、mz 四个参数，link1～link5 固定不动；输出聚合质量、质心、矩阵秩、原模型误差、训练误差和按点位五折留出误差。只有秩为 4、上游模型及力矩测量正确时，才能解释为可辨识的末端质量质心。转动惯量不由此次静态标定获得。`--fit-scope all` 可显式恢复旧版全臂 24 参数等效拟合。

link6 模式先从六关节实测力矩中减去固定 link1～link5 的重力贡献，再直接求解末端四参数。**没有向 620 g 或旧质心靠拢的正则项，也没有质量/质心改变量限制**；620 g 仅影响采集时使用的初始补偿。质量和质心允许明显变化，前五个 link 的参数完全保留。采集时固定使用初始模型，采完后再加载结果，不在途中更换补偿模型。

开始采集前打印计划回归矩阵秩和归一化条件数；拟合后再以实际反馈姿态评估。结果额外报告初始/最终质量、质量变化及毫米单位质心。若实际数据无法区分四个参数（秩不足 4），或算得非正质量，会保存诊断但标为 `usable=false`，不能当作有效模型加载。这属于辨识结果有效性检查，不是运动限幅。五折留出误差仍需要检查，满秩不等于标定准确。

## 6：库伦＋黏滞摩擦采集

```powershell
.\run_example.cmd 6_friction-calibration.py --execute --output logs\friction01 --joints 1 2 3 4 5 6 --span-deg 15 --speeds 5 10 20 40 --repeat 1
.\run_example.cmd 9_identify-offline.py friction logs\friction01
```

一次只动一个关节，其他关节保持；默认以启动反馈姿态为中心，往返中心 ±15°。`--center-deg` 可指定六关节中心。`--ramp .2` 是余弦加减速时间。初始增益可按跟踪情况调整；不自动沿用旧实验高增益。

离线分析使用 0.2 秒实际位置回归估速，检查 ±15% 速度偏差和轨迹线性残差，选中间 50% 角度区间；按实际六关节姿态配对正反向，减去实际 G(q) 后取力矩半差。多速度使用共同位置分箱，非负拟合 Fc、B，并输出无约束结果、残差和 bootstrap 区间。缺少足够速度或匹配覆盖的关节输出 null，不生成假参数。

## 7：静摩擦启动阈值

```powershell
.\run_example.cmd 7_static-friction-calibration.py --execute --output logs\static01 --joints 6 --repeat 3 --ramp-nm-s .1 --max-extra-nm 1
.\run_example.cmd 9_identify-offline.py static logs\static01
```

先定位并稳定，测试关节 Kp/Kd=0，其余关节保持；在 G(q) 上分别叠加正、负线性力矩斜坡。默认位移达到 0.3° 并持续 0.05 秒才确认起动，然后在实际姿态重新接入保持。达到指定斜坡终点仍未起动时标记 censored；反方向漂移标记 opposite_drift。两类都不伪装成有效静摩擦阈值。

分析输出正/负阈值、半差估计和半和偏置估计。结果包含重力残差、检测滞后及负载影响；建议多姿态重复。静摩擦结果独立保存，不自动作为零速持续推力加入自由拖动。

## 8：重力＋运动摩擦补偿

用户选定的 393.630 g 全数据 link6 结果已复制到 `profiles/gravity_link6_full284.json`。
历史 Follower 摩擦参数从用户提供的工作记录第 15.1 节导入到 `profiles/friction_legacy_follower.json`，记录了来源与哈希；未取得旧原始 CSV，未对新装配重新辨识。J2/J3 历史拟合残差较大。该文件明确标记为历史参数迁移，而非使用新重力模型重新拟合。运行日志保留实际参数和比例。

使用选定模型、六轴 60% 摩擦、持续运行（从 examples 目录）：

```powershell
.\run_example.cmd 8_gravity-friction-compensation.py --execute --identified profiles\gravity_link6_full284.json --friction profiles\friction_legacy_follower.json --friction-scale 0.6 --seconds 0
```

重力比例仍为 100%；摩擦原始系数只在运行时乘一次 0.6。近零死区和平滑过渡沿用例 8。

```powershell
.\run_example.cmd 8_gravity-friction-compensation.py --execute --friction logs\friction01\friction_identified.json --friction-scale .5 --seconds 15
```

要求 `--friction`；如采集使用 identified 重力模型，运行也传相同 `--identified`。默认比例 0.5，0.2 秒位置窗口估速，`tanh(v/0.1)` 平滑库伦项，|v|<0.03 rad/s 时摩擦归零。对应参数是 `--friction-scale`、`--friction-smooth`、`--friction-deadband`。仅使用文件里非 null 的关节；不会复制另一台臂的参数。当前实现不对黏滞项额外限幅，使用时应查看辨识文件中的有效速度范围和残差。

## 断点续采与离线重算

5、6、7 支持 `--resume --output 原目录`，其余实验参数必须与原运行一致。已完成点/行程跳过，中断行程从当前反馈重新定位并重跑。新尝试原始 CSV 另存；改变增益、模型、点位等实验条件时使用新目录。

```powershell
.\run_example.cmd 5_gravity-calibration.py --execute --output logs\gravity285_new --resume
.\run_example.cmd 9_identify-offline.py gravity logs\gravity285_new --output logs\gravity285_new\gravity_reanalysis.json
```

`metadata.json` 保存模型哈希、实际使用的重力参数和完整配置；`summary.json` 保存最近一轮结果；带时间戳 summary 保留每轮结果。CSV 含命令、反馈、MCU 时间、序号、反馈年龄、电机模式和故障字段。离线摩擦分析选每个行程最后一次完成尝试，不拼接中断轨迹。

## 启停的确切含义

示例会 claim 会话、关闭 example2 的 zero-MIT 覆盖、先写第一条有效命令再 active。全程不发送灵巧手目标、不切换灵巧手模式。

现有固件 operation 4/2 只退出 active/释放会话，电机会保留最后 MIT 命令。默认 `--exit-mode hold` 在退出前发送当前姿态＋指定 PD＋当前重力值，然后释放；这是电机保持命令，不是机械抱闸。`--exit-mode last` 保留实验最后命令。网络断开时无法保证退出命令送达；summary 会记录失败，不能把程序退出当作电机静止证据。

## 本轮验证状态（2026-10-30）

### 指定三个姿态补采与90%权重优化

例11的三个绝对编码器角度（度）：

```
[-43.7,75.4,71.3,-50.8,-41.2,-2.6]
[-8.9,130.8,97.1,-42.6,53.1,14.1]
[-1.4,156.1,144.4,-58.7,77.5,-30.6]
```

三个姿态按以上顺序循环三轮，共9次采样。每次8秒五次插补、4秒等待、3秒采样、2秒保持，理论153秒。Kp=[24.48,61.2,61.2,12.24,12.24,12.24]，Kd=[1.2,1.92,1.92,0.48,0.48,0.48]。使用选定393.63g重力模型作为采集前馈，不加摩擦补偿。仍记录实际六轴位置/速度/力矩，目标仅用于运动和分组；采集时不自动修改模型。

从 host 目录运行：

```powershell
.\examples\run_example.cmd 11_priority-pose-capture.py --execute --output logs\priority3_001
.\examples\run_example.cmd 12_priority-gravity-fit.py logs\priority3_001 --output logs\priority3_001\weighted90_001
```

第二条在采完后运行，只处理文件，不连接机械臂。例11的9次采样不会触发通用例5的10点自动拟合门槛；由例12与旧285点数据合并分析。

默认新数据总权重0.9、原合格数据总权重0.1；三个新姿态各占0.3，同一姿态内按合格重复次数均分。每个目标至少需要两次合格重复，以比较留出重复采样误差；缺失目标组不会被静默忽略。可用 `--priority-share` 改权重，用 `--original` 指定原数据目录。

拟合与全部 RMSE 仅使用 J2–J6 力矩，J1 力矩不进入目标函数或评估；姿态计算仍保留六个关节角度。报告中的关节数组按 `joint_ids: [2,3,4,5,6]` 排列。旧报告含 J1 的总体 RMSE 不可与新报告直接比较。

输出 `link6_identified.json`、`all_identified.json` 和 `comparison.json`；具有正质量的候选还导出对应 URDF。报告包含重点姿态、原数据、加权总体及留出重复采样 RMSE，还有各 link 质量/质心的变化。link6方案完全固定前五个link；全臂方案调整24个质量/一阶质量矩，以当前模型为参考选归一化最小改动等效解，通常不唯一。转动惯量保留原值，不能用于声称完成全动态标定。三个新姿态不足以独立标定整臂；留出重复采样只是同姿态可重复性评估，不是未知姿态验证。候选不会自动替换默认模型。

已通过数值对照、合成重力/摩擦辨识、轨迹导数连续性、六入口模拟反馈流程、断点续采及失败清理测试。另有真实 loopback UDP 测试，实际经过 robot_sdk 编解码、可靠 RPC、状态接收和五参数发送。

本机物理网络预检：以太网 192.168.10.1/24；5 秒内未收到 192.168.10.11 状态，指定源地址 ping 也未获得开发板回复，邻居解析未完成。因此**这些功能的实机动作和标定精度尚未验收**，当前未生成真实机械臂的新标定参数。

复现测试：

```powershell
.\run_example.cmd test_arm_examples.py
.\run_example.cmd test_udp_experiment.py
```
