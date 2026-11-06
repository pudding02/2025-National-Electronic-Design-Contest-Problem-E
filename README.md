# 电赛E题 — MaixCAM2 双轴云台追踪系统 (CV/YOLO + 双步进/双舵机)

基于 Simple_Tracking_Device 原项目思路的完整复刻：K230-CanMV **单板**完成视觉检测与云台控制，
双轴执行器均为 **Emm42_V5.0 闭环步进**（42 电机）——左右 yaw 与上下 pitch 都走串口「速度模式」（两轴同逻辑）。

> 历史沿革：原版跑在 MaixCam (MaixPy) + STM32F407 中继架构上（K230→STM32→云台）；
> 本仓库已演进为 **K230 直控**（不经 STM32），且 pitch 轴从 FashionStar 总线舵机改为第二台 Emm42 步进。

## 系统架构

```
┌─────────────────────────┐
│       K230-CanMV        │
│  视觉检测 (CV / YOLO)    │
│  + 云台控制 (双步进)      │
│                         │
│  MIPI CSI 摄像头         │
│  LCD ST7701 800×480     │
│  激光 GPIO(可选)         │
└──────┬──────────┬───────┘
       │ UART3    │ UART2
       ▼          ▼
  驱动器①      驱动器②
  Emm42_V5.0   Emm42_V5.0
  (速度模式)    (速度模式)
       │          │
       ▼          ▼
   42 步进      42 步进
   [左右 yaw]   [上下 pitch]
```

- 完整接线见根目录 **`接线图.md`**（引脚清单 / 驱动器预置表 / 电源共地 / 排障）
- 视觉→执行器唯一接口：`gimbal_send(dx, dy, dist, status)`（在 `k230/main.py` 内）

## 与原项目的关键区别

| 维度 | 原项目 (MaixCam) | 本复刻 (K230 双步进) |
|------|:-----------:|:------:|
| 视觉芯片 | K210 (MaixCam) | K230 (庐山派) |
| 检测方式 | YOLO NN 模型 | 经典CV + YOLO KPU 双模式 |
| 云台链路 | MaixCam→STM32→执行器 | **K230 直控**（无 STM32） |
| 左右轴 (yaw) | 舵机/开环 | Emm42 步进 · 速度模式（\|dx\|→RPM） |
| 上下轴 (pitch) | 舵机/开环 | Emm42 步进 · 速度模式（与 yaw 同逻辑） |
| 串口协议 | `0x88*8` 固定字节 | Emm42 帧（固定 0x6B 校验） |
| 分辨率 | 448×448 | 640×360 (16:9) |
| 帧率 | ~25fps | ~45fps (CV模式) |
| 配置方式 | 硬编码 | `config.json` 配置文件 |

## 目录结构

```
复刻/
├── README.md                   # 本文档
├── CLAUDE.md                   # 给 AI 助手的项目说明
├── 接线图.md                    # 接线与驱动器预置（唯一权威）
├── k230/
│   ├── main.py                 # K230 CanMV 主程序（CV+YOLO 检测 + 双步进控制）
│   └── config.json             # 运行时配置（唯一调参面）★
├── skill/                      # 方法论文档（参考用，非运行时）
│   ├── 01_方法论文档_视觉识别与云台控制全解析.md
│   ├── FASHIONSTAR-SERVO-K230-SKILL.md    # 已弃用：pitch 舵机方案存档
│   ├── K230-DRONE-AI-SKILL.md
│   ├── K230-YOLO-TRAIN-DEPLOY-SKILL.md
│   └── LUSHAN-PI-K230-PINOUT-SKILL.md     # 40Pin 引脚速查
├── 云台/                       # 硬件资料（驱动器说明书 PDF / 上位机 / 历史接线图）
└── tools/                      # 主机侧回归测试（不参与部署）
    └── test_*.py               # 9 套，共 285 例
```

## 一、K230 视觉端部署

### 1.1 硬件

- 庐山派 K230-CanMV 开发板（标准版 1GB）
- MIPI CSI 摄像头（板载 GC2093）
- ST7701 LCD（板载，800×480）
- 2× Emm42_V5.0 闭环步进驱动器 + 2× 42 步进电机（yaw / pitch）
- 可选：激光模块（GPIO 控制）

### 1.2 部署步骤

```bash
# 1. 将 k230/main.py 和 k230/config.json 复制到 SD 卡根目录
cp k230/main.py /sdcard/main.py
cp k230/config.json /sdcard/config.json

# 2. （YOLO模式）复制 kmodel 到 SD 卡
mkdir -p /sdcard/model
cp model.kmodel /sdcard/model/

# 3. SD 卡插入 K230，CanMV IDE 中打开 main.py 运行
```

### 1.3 上电流程

1. **上电前把云台俯仰摆到行程中部**（步进断电无保持力，pitch 可能因重力下垂；两轴都建议机械居中安装）
2. 上电后核对串口第一行：`actuator:` 两轴均应 `online`
3. 首次联调先单轴自检：config 里 `motor.test_mode` / `pitch_motor.test_mode` 置 `true`，不跑视觉只看动作

### 1.4 设备连接与远程登录（MaixCAM2）

| 项 | 值 | 说明 |
|---|---|---|
| 连接方式 | MaixVision（USB 网卡或同一 WiFi） | 左下「连接」，自动发现 `maixcam*` |
| 设备 IP（本机） | USB 网卡 `10.150.102.1` | 接 USB 线就有，最稳；WiFi 地址 `10.55.95.113` 会随网络/DHCP 变，以 MaixVision 连接页或设备 `hostname -I` 为准 |
| SSH 账号 | `root` | MaixCAM/MaixCAM2 默认账号 |
| SSH 密码 | `root` | 出厂默认值；本机曾被改过，2026-11-05 已重置回 `root` |

```bash
# 电脑上直连设备(密码 root); USB 网卡优先, 不依赖 WiFi
ssh root@10.150.102.1
ssh root@10.55.95.113        # 走 WiFi 时(IP 会变, 先确认)
# 设备上查自己的 IP
hostname -I
```

- 密码只以哈希形式存在设备的 `/etc/shadow`，没有明文可查；忘了密码只能重刷系统镜像恢复默认。
- 远程跑主程序：`scp k230/main.py k230/config.json root@<IP>:/root/` 后 `ssh root@<IP> "cd /root && python3 main.py"`；**别和 MaixVision 同时跑同一个程序**，两者会抢摄像头/UART。
- 该密码只用于本地调试，不要用在对外网络上。

## 二、配置文件说明 (config.json)

所有运行参数集中在 `k230/config.json`，改完重启生效。以 `_` 开头的键是注释，程序自动跳过。

主要段落：

| 段 | 作用 |
|---|---|
| `detection` | 检测模式（cv / yolo）与跳帧 |
| `camera` / `display` / `center` | CSI 端口（`csi_id`）、输出分辨率（16:9 全幅）、显示偏移、追踪中心点（含激光偏移 `laser_offset_x/y`）|
| `center_calib` | **触屏校准 + 调参页**：校准页按钮移动绿色校准点（=追踪收敛点，即 `center.laser_offset`）、`T/C` 追踪/画圈切换、`●` 隐藏、`S` 保存写回；`P` 进调参页（`Y/P` 切轴、`项` 选参数、`+`/`-` 实时调、`S` 写回，见 4.3）|
| `rectangle` | CV 矩形检测阈值（现场光照变化主要调这里）|
| `yolo` | KPU 模型路径 / labels / input_size（见第四节规则）|
| `motor` | **yaw 轴**：Emm42 速度模式（RPM 区间 / 死区 / 方向 / 使能）；V7.9 PID 参数 `pid_enabled` / `pid_kp` / `pid_ki` / `pid_kd`（见 4.3）；V8 卡尔曼滤波 `kf_*`（见 4.8）与堵转监控 `monitor_ms`（见 4.9）|
| `pitch_motor` | **pitch 轴**：Emm42 速度模式（与 yaw 同构：min_rpm/max_rpm/dy_full_px/死区/方向）；同含 `pid_*` 键 |
| `red_target` | **红环检测**（与矩形检测同帧联合进行，画圈模式用）：红色掩膜/环直径/空心验证阈值、ROI 外扩、半径修正；V8.5 总开关 `enabled`（false=完全跳过）|
| `search` | **上电搜索**：未捕获目标时 yaw 单方向匀速旋转找靶（enabled/rpm/direction/max_ms）；V15 姿态模式下改由 `turns`（圈数，0=一直找）与 `dps`（世界角速度）驱动的**单方向按圈扫**（见 4.10）|
| `circle` | **画圈运动**：沿红环匀速转圈的角速度/方向/误差门控（speed_deg_s/direction/pause_px）；V8 环丢失记忆兜底 `fallback_ms`、V8.5 几何主轨 `radius_ratio`/`calib_frames`（V8.6 渐进标定默认开，见 4.6）|
| `mode_pin` | **画圈模式引脚**（GPIO49/Pin3，可选备用，默认关闭）：被供电=画圈；默认由触屏 `T/C` 切换（引脚开启时以引脚为准）|
| `attitude` | **V15 姿态稳定**（默认 `enabled=false`，关掉即 V14 行为）：内环拍长/预算、角度环 P、速率环积分、死区、`yaw_sign`、`px_per_deg`(0=自动反算)、外环增益、`hold_ms`、发散保护、扫掠触发阈值。现场只需动 `yaw_sign`/`angle_kp`/`rate_ki`/`deadband_dps`（见 4.10），触屏调参页有 `att`/`AKp`/`ADz` 三项 |
| `debug` | 打印与 GC 节奏 |

## 三、检测模式（CV / YOLO）

### 3.1 CV 模式（默认，推荐矩形靶标）

经典计算机视觉矩形检测，使用 OpenCV 白色掩膜 + 轮廓筛选 + 黑边/白心验证。

- 优点：帧率高（~45fps）、无需训练模型、参数可现场调优
- 缺点：背景杂乱时可能误检
- **适用场景：黑背景上的白色矩形靶标（白底黑边）**

### 3.2 YOLO KPU 模式

使用 K230 KPU 进行 YOLOv8 神经网络推理，支持 detect 与 segment 两种任务。

- 前提：需预先训练并转换 kmodel（参考 `skill/K230-YOLO-TRAIN-DEPLOY-SKILL.md` 与 `skill/K230-DRONE-AI-SKILL.md`）
- 优点：可检测任意训练过的目标（无人机、水果、手势…），对复杂背景鲁棒
- 缺点：帧率较低（~15-25fps）、需提前训练模型

#### ⚠️ 关键配置规则（违反则不出框或误判满天飞）

| 规则 | 说明 |
|------|------|
| **input_size 必须等于训练 imgsz** | 模型名含 `320` → 填 `[320,320]`；不匹配会导致坐标错乱/不出框 |
| **model_path 和 task 必须配对** | `_seg_` 模型 → `"task": "segment"`；`_det_` 模型 → `"task": "detect"` |
| **labels 必须与训练时完全一致** | 顺序、数量、名称都不能错 |
| **rgb888p_size 随 task 变化** | segment → `[320, 320]`；detect → `[640, 360]` |
| **GC2093 是 16:9 传感器** | 本条针对 YOLO 输入参数（`rgb888p_size` 等）：不建议设 4:3，否则初始化可能失败。CV 模式采集配置另见「故障排查」（1280×720 提示 + `crop=True` 裁切）|

## 四、云台控制（双步进）

两轴均为 Emm42_V5.0 闭环步进，协议来自 `云台/Emm_V5.0步进闭环驱动说明书Rev1.3.pdf`。
驱动器侧预置（校验 0x6B / 波特率 / ID / UART_FUN / 应答）见 `接线图.md`。

### 4.1 左右轴 (yaw) —— 速度模式

`|dx| ≤ deadband_px` → 停转；之后从 `min_rpm` 线性升到 `max_rpm`（`|dx| ≥ dx_full_px` 时满速）。

**丢靶预判**（默认关闭；先单独验证、再启用）：目标丢失时若最后偏差 `|dx| ≥ coast_edge_px`（默认 160，即最后在画面左右两侧），
不立即停，而是朝最后方向按最后偏差映射的转速续转 `coast_ms`（默认 800ms）去找（无目标追踪）；
期间重新捕获立即恢复常规追踪，超时则停。

单独验证：`[motor].coast_test` 置 `true` 上机跑——不跑视觉，自动模拟左右两侧丢靶 + 中途重捕获，观察云台续追方向/时长/停止
（串口打印 `COAST start` / `COAST stop`；与 `test_mode` 不要同时开，同开时 `coast_test` 先执行）；
验证满意后把 `coast_enabled` 改 `true` 正式启用。
`coast_enabled=false`（默认）即「丢靶即停」，不影响日常追踪。

### 4.2 上下轴 (pitch) —— 速度模式（与 yaw 相同逻辑）

`|dy| ≤ deadband_px` → 停转；之后从 `min_rpm` 线性升到 `max_rpm`（`|dy| ≥ dy_full_px` 时满速）；丢靶停转。

> 历史：曾试行位置模式（角度记账 + 绝对位置目标，需上电摆中位校准 0 点），实测驱动器②不工作，已改为速度模式。

### 4.3 PID 控制（V7.9，可选）与触屏调参

两轴默认仍走 4.1/4.2 的线性映射；把对应轴 `pid_enabled` 置 `true`（或在触屏调参页把该项调到 ON）后切换为**速度式 PID**：

- `u = Kp·e + Ki·∫e·dt + Kd·de/dt`（e = 像素误差，u = RPM；输出限幅到 `max_rpm`；|u| 小于 `min_rpm` 时抬到 `min_rpm` 保步进起步扭矩）。
- 死区内停转并清 PID 状态；丢靶 / 上电搜索 / 丢靶预判期间同样清零——不跨状态积累积分（防恢复追踪时突跳）。
- **抗饱和**：输出饱和且误差仍指向饱和方向时冻结积分；首帧 dt 按 33ms 兜底、异常长帧截到 0.5s（防微分放大）。
- 参数：`pid_kp` 比例（RPM/px，默认 0.28=yaw / 0.14=pitch，≈原映射斜率）、`pid_ki` 积分（消恒差，现场试 0.01~0.1 起）、`pid_kd` 微分（抑过冲，试 0.01~0.05 起）。
- **触屏调参页（实时生效，免重启）**：校准页按 `P` 进入；`Y/P` 切轴；`项` 按钮循环选项（PID 开关 / Kp / Ki / Kd / Max / Min / Dead / Acc / Full）；`+` / `-` 步进（按住连发），改动**下一帧即生效**；`S` 写回 config.json（`.bak` 备份 + 写后读回校验）；`X` 返回校准页。
- 丢靶预判（coast）与上电搜索不受 PID 开关影响，仍按原映射转速执行。

### 4.4 调参速查

| 现象 | 调整 |
|------|------|
| 云台振荡/过冲 | 降对应轴 `max_rpm` / 调大 `deadband_px` |
| 对准慢 | 先调大 `acc`（V8.6 默认 250；公式：每 +1RPM 用时 (256-acc)×50μs，见 10.3）；再调大 `max_rpm` / 调小 `dx_full_px`(yaw)、`dy_full_px`(pitch) |
| 起步/换向迟钝（追起来才跟上） | 调大 `acc`——起步与换向的转速建立时间是当前最大延迟源；`acc=0`=无斜坡直接跳变（极限选项，现场试）；仍不够再调大 `max_rpm` |
| 低速"哼着不转" | 对应轴 `min_rpm` 调大（步进低速扭矩小） |
| 中心附近抖动 | 调大对应轴 `deadband_px` |
| 方向反了 | `motor.reverse` / `pitch_motor.reverse` 置 true |
| 目标甩出画面后跟丢 | 先 `coast_test` 自检预判，确认行为后 `coast_enabled` 置 true；力度不够再调大 `coast_ms` / 调小 `coast_edge_px` |
| 丢靶后乱转/干扰追踪 | `coast_enabled` 保持/置 false（回「丢靶即停」）；或调小 `coast_ms` |
| 跟踪不平滑 | 增大检测端滤波/跳帧参数 |

### 4.5 目标识别：矩形追踪 / 红环画圈（V7）

矩形检测与红色（红环）检测**每检测帧联合进行**；早期版本的红点（靶心）瞄准已删除（与矩形识别冲突），追踪以**矩形中心**为准：

| 模式 | 目标 | 串口目标源 |
|---|---|---|
| 追踪（默认） | **矩形中心** | `OK [rect]` |
| 画圈（触屏 `T/C` 切换，见 4.6） | **红环/几何合成圆**（环上 θ 点） | `OK [geo]`（几何）/ `OK [ring]`（检测） |

无有效目标 → 丢靶逻辑（coast/停）；上电未捕获期 → 自动搜索（见 4.7）。

### 4.6 画圈模式（触屏 T/C 切换；引脚可选）

- 画圈模式下红环可见 → 光斑沿红环**连续匀速画圆**（`circle.speed_deg_s` 角速度、`direction` 方向；光斑离目标点超过 `pause_px` 时暂停 θ 推进等伺服追上）。
- 红环暂不可见 → 默认退为矩形追踪瞄准（环恢复即续圈）；开启 `circle.fallback_ms`（推荐 800）后，环刚丢的一段时间内用**记忆半径 + 矩形中心**继续画圈（串口打印 `CIRCLE fallback`），窗口过后才退矩形追踪。
- **几何主轨（V8.5/V8.6，推荐——尤其激光干扰红环检测的场景）**：`circle.radius_ratio > 0` 时，画圈的环几何**完全不再消费红环检测**——圆心=框中心（同心居中先验）、半径 = `k × √(框面积)`。免疫激光扫环的自干扰、无帧间抖动、无丢失中断。k 的获取（免物理标定，比例制对距离变化自动成立）：
  1. **渐进标定（V8.6 默认开）**：`circle.calib_frames: 30` —— 开机后环检测**成功的帧**自动入池（追踪模式下激光不扫环，收集最快；画圈模式也在持续收集），满 **6 个样本即实时可用**（串口打印 `CIRCLE autocalib ready: k=0.xxxx (n=6) 已实时生效`），集满 30 个收敛（打印 `CIRCLE autocalib: k=... 收敛 — 建议写入 config`）；
  2. 把收敛值填入 `circle.radius_ratio`（永久生效，优先级高于运行时值）；不填则每次运行自动重标（数秒内可用）。
  - 收不到样本时的人工兜底：把靶面摆正到画面中央，量黑框像素宽×高与红环像素直径，算 `k = r ÷ √(w×h)` 手填 `radius_ratio`。
  - 配套：`red_target.enabled: false` 可整体关断红环检测（省算力/彻底免疫；画圈需已填 `radius_ratio` 或本次运行已就绪）。
  - 串口/OSD 目标源区分：几何路径 `[geo]`，检测路径 `[ring]`，记忆兜底 `CIRCLE fallback`。
- **OSD 圆语义（V8.6）**：画圈时屏上画圆——**橙色**=几何合成圆（k 路径）、**蓝色**=本帧检测到的红环、**青色**=记忆兜底圆；黄色小圈=当前目标点。画圈激活但无环源（退化追踪）时屏上显示 `CIRCLE: no ring/k`、串口低频提示。
- **切换方式（默认）**：触屏顶中 `T/C` 按钮点击切换（追踪⇄画圈），串口打印 `MODE->.. (touch)`。
- **切换方式（可选备用）**：`config.json` 的 `mode_pin.enabled=true` 时改用 GPIO49 跳线：**三脚跳线**（中间=GPIO49/Pin3，两侧=3.3V/Pin1 与 GND）：拨 3.3V=画圈、拨 GND=仅追踪；**悬空不可信**（I2C0 引脚板载上拉情况以实测为准）。此时触屏 `T/C` **故意失效**（以引脚为准），上电串口打印 `mode pin: GPIO49 level=..` 供核对。

### 4.7 上电搜索

- 开机后**未捕获过任何目标**时，yaw 按 `search.rpm` 单方向匀速旋转找靶；捕获目标（矩形或红环）即停并转入追踪。
- 一旦捕获过目标（发出过 track），搜索**永久退出**——后续丢靶行为与原来一致（coast/停），不会被搜索干扰。
- `search.max_ms=0` 表示一直找到为止；>0 为搜索上限（超时停转，不再重启）。

### 4.8 卡尔曼滤波（V8，可选；默认关）

对视觉误差做 2 状态（位置+速度）卡尔曼滤波——移植参考云台（robot_cmd.c VisionKalmanFilterInit）的设计：

- **开启**：对应轴 `kf_enabled=true`（默认 false=原样直通，行为不变）。开启后误差更平滑（检测抖动小）；`kf_lead_s>0` 时用速度估计做**提前量前馈**（补偿检测+显示帧延迟；动态目标拖尾明显时试 0.05~0.1，0=关）。
- **参数**（每轴一组）：`kf_q_pos`/`kf_q_vel` 过程噪声（调大=跟得快、平滑弱）、`kf_r` 观测噪声（调大=更平滑、更滞后）、`kf_err_limit` 输出限幅(px)。
- **状态清理与 PID 同步**：丢靶/搜索/死区停转时一并清空（防恢复追踪时速度状态突跳）。
- **与 PID 正交**：KF 只作用在"误差输入"，PID/线性映射的选择照旧；建议先单独开 KF 观察平滑收益，再决定要不要叠 PID。

### 4.9 堵转监控与读取命令（V8，可选；默认关）

挖掘 Emm42 的读取类命令（命令码来自张大头官方例程读取命令表；帧式与读电压 0x24 同构）：

- **boot 探测增强**：`actuator:` 行在电压后追加 `rpm=N`（读一次实时转速 0x35，上电应接近 0）——同时验证读取链路是否通。
- **堵转监控**：`monitor_ms>0`（建议 2000）后，主循环每 N ms 读一次实测转速；「有速度命令但实测≈0」打印 `WARN: possible stall`（**仅告警，不影响控制**）。
- **清堵转**：驱动器因撞限位/卡死触发堵转保护后，`clog_reset_on_boot=true` 可上电自动发清堵转命令（0x0E52）恢复。
- **首次上机核对**：读取类响应字节段为**预期格式**（0x35 预期 6B / 0x36 预期 8B，位置语义待核）；先 `raw_probe(0x35)` / `raw_probe(0x36)` 捕获实测核对，再把样本补进测试固化。无应答/校验错返回 None，不阻塞主循环。

### 4.10 姿态稳定（V15，板载 IMU 内环 + 视觉外环）

针对"云台底座被快速旋转（手转/转台），摄像头要跟住靶心不丢靶"。板载 IMU（MaixCAM2 = **LSM6DSOW**，IIC1/0x6B，acc+gyro，**无磁力计**）与相机在**同一块板、同一刚体**上，免接线。

**为什么不直接"测到扰动就反向补偿"**：IMU 绑在相机上，陀螺测到的是
`ω_cam = ω_base + m_motor` —— 底座扰动与云台自己的跟踪转动在传感器上**完全混在一起**，无法分离。所以不能用前馈猜扰动，只能闭环控制**相机自身的绝对姿态**：

```
视觉(45fps)                          内环(≈650Hz, 帧间)
  dx ──► 外环 ──► θ_ref ───────────► att_step ──► 整数RPM ──► Emm42(yaw)
         px→°        ▲                  │
         增量修正      └──── θ_imu ──────┘  ▲
                                      板载IMU
```

内环每拍：`ω_base_hat = ω_cam - 6·rpm_prev`（扰动观测器反推底座转速）→ `ω_des = angle_kp·(θ_ref-θ_imu)` → 指令 = 积分器输出。**只在 RPM 变化时写串口**（1RPM=6°/s 的量化天然限流）。

**零点与靶心方位**：无磁力计 → yaw 无绝对基准，所以**上电位置即 0 点**；靶心方位 = 首次锁定时 `θ_imu + dx/px_per_deg`，此后它就是被保持的世界方位。不需要机械原点，也不需要磁力计——考核窗口是"两帧之间的 22ms"，陀螺比例误差 1% 在那段时间只贡献 0.04°。

**px_per_deg 免新增标定**：由 `[distance]`（ref_area_px/ref_distance_mm）与 `[circle]`（rect_w_cm/rect_h_cm）自动反算焦距 `f = 宽像素×距离/物理宽`，再 `px_per_deg = f·π/180`。当前标定值反算 ≈ **11.5 px/deg**，即验收线 0.5° ≈ **5.8px**。要覆盖就填 `[attitude].px_per_deg`。

**丢靶链路（V15 改造）**：`lock`（视觉锁定）→ 丢靶 `hold`（内环把相机钉在 θ_ref 上，`hold_ms` 内**靶标回画面立刻续跟**）→ 超时 `search`（**单方向按圈扫**，用 IMU 积分计 360°，`search.turns=0` 不限圈数、N=转满 N 圈停且不再重启）。搜索时的扫掠是"恒定世界角速度"——底座同时在转也不影响真实扫过的角度。

**仿真实测（`tools/test_attitude_loop.py`，默认增益、1 拍延迟、1RPM 量化）**：

| 扰动 | 峰值偏差 | 稳态偏差 | 结论 |
|---|---|---|---|
| 0→200°/s 阶跃 | 0.82°（9.4px） | **0.29°（3.3px）** | 稳态远优于 0.5° 验收线 |
| ±100°/s 正弦 @0.5Hz | 0.28°（3.2px） | 0.27°（3.1px） | 全程在验收线内 |
| 5Hz 手抖 ±30°/s | 0.17°（2.0px） | 0.17°（2.0px） | — |
| 陀螺 1% 比例误差 | 0.82° | 0.29° | 几乎无影响（印证"帧间窗口"论证） |
| 陀螺 0.5°/s 读数噪声 | 0.82° | 0.27° | 不引起抖动 |
| 延迟放宽到 3 拍 | 1.72°（19.7px） | 0.14°（1.6px） | 仍收敛 |
| **`yaw_sign` 配反** | 发散 | — | 由 `fault_deg` 保护在 300ms 内自动降级 |

**增益稳定边界**（仿真实测，`rate_ki` vs 传输延迟）：延迟 ≤2 拍时 0.7~0.8 可用；**延迟 3 拍时 ≥0.6 会振成 2~6° 的极限环**，0.55 仍收敛（稳态 0.13°）。所以默认 `rate_ki=0.55`，给未知延迟留余量——现场若出现**低频来回摆动**，第一件事是把 `rate_ki` 往下调。

**上机顺序（每步都能单独回退）**：

1. 上电看串口 `att:` 行：确认 IMU 型号/`mag=False`；再手转底座看 `ATT th=` 与 `w=` 符号是否一致。
2. `attitude.enabled=false` 先观察：OSD 左下 `ATT θ d(err) z(ω) 模式`、串口 1Hz `ATT th=... ref=... err=...`，手转底座看 θ 与肉眼转角一致（误差 <1°），据此定 `yaw_sign`。
3. 开 `attitude.enabled=true`，先把 `outer_gain=0`（纯姿态保持）：手轻转底座，画面里靶标应**纹丝不动**。若越纠越偏 → `yaw_sign` 取反；若低频来回摆 → 降 `rate_ki`。
4. `outer_gain` 回 0.8：手转 ±90°、来回摆、连续单向转，看串口 `err` 与 OSD 的靶心偏移是否 ≤ 0.5°（≈6px）。
5. 遮挡靶标 2s（<`hold_ms`）→ 应保持方位并在恢复后 3 帧内续跟；遮挡 >`hold_ms` → 应打印 `HOLD timeout -> SEARCH` 并进入单方向扫掠。

**代价**：强扰动/扫掠/保持时，帧间预算（`tick_budget_ms`，默认 6ms）给内环，视觉帧率从 ~45fps 降到 ~30fps。静止追踪时内环每帧只跑 1 拍，帧率不变。

**注意**：姿态模式用 `[attitude].acc`（默认 255 ≈0.05ms/RPM，让指令在 1 拍内建立），`[motor].acc` 只在关闭姿态时生效；`[attitude].min_rpm` 默认 1（=6°/s），不要沿用 `[motor]` 的 5（=30°/s），否则小幅纠偏只能靠长通断硬凑、抖动明显。

## 四·补、双舵机模式（V16，可选；当前 config 默认开启）

把执行器从「双 Emm42 步进（速度模式）」换成「FashionStar FSUS 总线舵机（位置模式）」，
由 **MaixCAM2 通过 UART2 单总线直控**（`maix.uart` + `pinmap`，不经过 STM32/庐山派 K230）。
视觉检测链、画圈、激光标定、触屏校准全部不变；`gimbal_send(dx,dy,dist,status)` 接口不变。

**接线**（详见 `接线图.md` 第零节；`云台/舵机接线图.png` 是旧庐山派 K230 版，已作废）：

```
MaixCAM2 B0(UART2_TX) --[1kΩ]--+-- 舵机① yaw   ID=0
MaixCAM2 B1(UART2_RX) ---------+-- 舵机② pitch ID=1   (信号并联)
GND 共地；舵机 V+ 独立电源（勿用板载 5V）
```

![MaixCAM2 双舵机接线](云台/舵机接线图_MaixCAM2.png)

**控制**：

- 每帧一帧 `cmd25` 同步多圈帧（子命令 13），两轴绝对目标角同时下发；
- 外环 `e → KF(可选) → 增量 PID(Kp/Ki/Kd/Kff) → 目标角累加`，舵机内部位置环负责快速跟踪；
- 死区 3px、每帧最大增量 `max_step_deg`、软限位、积分分离/抗饱和；
- 上电读当前角并以当前角为目标（不跳变）；丢靶保持 `hold_ms` → 有界往返搜索；
- 与步进互斥：`servo.enabled=true` 时不初始化 Emm42；回滚把 `servo.enabled=false`、`motor.enabled/pitch_motor.enabled=true`。

**关键配置**（`k230/config.json → [servo]`）：

| 键 | 默认 | 说明 |
|---|---|---|
| `enabled` | true | 舵机路径总开关 |
| `yaw_id/pitch_id` | 0/1 | 实际相反就互换 |
| `zero_auto` | true | 软件零点：启动时把当前姿态记为逻辑 0°（不写舵机 EEPROM，断电不保持） |
| `yaw_zero_deg/pitch_zero_deg` | 0/0 | `zero_auto=false` 时的固定零点偏移（原始多圈角，如 pitch=-60） |
| `yaw_sign/pitch_sign` | 1/1 | 越跟越偏就取 -1 |
| `yaw_min/max` | ±32000° | **yaw 不做软件限位**（连续旋转，≈±88 圈，协议 int32×0.1 上限 ±32767.9°）；没有滑环时线会绕 |
| `pitch_min/max` | ±30° | 参考云台也是 ±30 |
| `yaw_kp/pitch_kp` | 0.55/0.42 | (°/s)/px；100px≈55/42°/s（0.18 太慢，实测 3 倍增益仍稳） |
| `yaw_ki/pitch_ki` | 0.15/0.12 | 消静差/重力下垂；先 0 再逐步加 |
| `yaw_kd/pitch_kd` | 0.012/0.010 | 抑超调；用 KF 速度，噪声大就减 |
| `yaw_kff/pitch_kff` | 0.05/0.04 | 速度前馈；需 `kf_enabled=true`（用 KF 速度） |
| `max_dps` | 260 | 外环角速度上限（原 150 偏慢） |
| `max_step_deg` | 10 | 每帧目标角最大增量（30fps≈300°/s） |
| `deadband_px` | 3 | 死区；3px≈0.26° |
| `interval_ms` | 25 | 舵机走完一帧目标的周期 |
| `hold_ms/search_*` | 1500 / ±120° | 丢靶保持 → 有界搜索 |
| `kf_enabled` | false | 首次上机先关，稳定后再开 |
| `test_mode` | false | true=只做 ping/读角/±5°点动/同步帧自检 |

**首次上机**：

1. 接好单总线/独立电源/共地，确认两舵机 ID=0/1、115200；
2. `servo.test_mode=true` 跑一次：看 ping、读角、两轴 ±5° 点动、cmd25 同步；
3. 确认方向（目标在右/下时云台应朝右/下）→ 不对改 `yaw_sign/pitch_sign`；
4. 确认机械行程 → 把 `yaw_min/max` 设到行程内（先 ±60°，再逐步放大）；
5. `test_mode=false`，先 `pid_enabled=false`（纯 P）调 `kp`，能跟不振后加 `kd`、`ki`、`kff`，最后开 KF。

**注意**：舵机模式的触屏 `P` 调参页暂未映射（按下会提示改 config）；参数请直接改 `k230/config.json` 的 `[servo]` 段并重启。步进模式的触屏调参页不受影响。

## 五、激光标定

激光与摄像头光轴不同轴时，标定 `center.laser_offset_x/y`（激光在画面中心右/下为正）。
含义：把"追踪收敛点"偏移到激光实际落点处——追踪时目标停在这个点，激光即可命中。

### 5.1 触屏现场标定（推荐，免电脑）

运行 `main.py` 后画面四缘出现触摸按钮：

| 按钮 | 位置 | 作用 |
|---|---|---|
| `▲ ▼` | 左缘 | 绿点（校准点）上 / 下移动 |
| `◀ ▶` | 右缘 | 绿点左 / 右移动 |
| `T / C` | 顶中 | 追踪 / 画圈模式切换（显示当前模式：T=追踪绿 / C=画圈橙；串口打印 `MODE->.. (touch)`）|
| `●` | 顶中 | 隐藏绿点+按钮（画面干净）；再触摸屏幕任意处唤出 |
| `S` | 顶中 | 当前偏移写回 `config.json`（自动备份 `.bak`、写后校验）|
| `P` | 顶中 | 进入**调参页**（PID / 云台参数，见下表）|

单点一步 `step_px`（默认 4px），按住不放连发（间隔 `repeat_ms`）；绿点即时生效于追踪/画圈。
操作：把靶标与激光同处画面，用按钮把绿点拖到"激光应命中的位置"，观察云台收敛，满意后按 `S`——
串口打印 `calib: save OK saved (x,y)`，重启后仍生效。

**调参页（`P` 进入 / `X` 返回）**：调 PID 与云台参数，实时生效（现场调完按 `S` 写回）：

| 按钮 | 位置 | 作用 |
|---|---|---|
| `+` / `-` | 左缘 | 当前参数值 +/- 一步（按住连发；步长按项：Kp±0.02、Ki/Kd±0.01、Max/Min/Dead±1、Acc±5、Full±10）|
| `项` | 右缘上 | 循环切换参数项：PID 开关 → Kp → Ki → Kd → Max → Min → Dead → Acc → Full |
| `Y / P` | 右缘下 | 切换调节的轴（Yaw / Pitch）|
| `S` | 顶中 | 全部 18 项写回 `config.json`（`.bak` 备份 + 读回校验）|
| `X` | 顶中 | 返回校准页 |

页面底部实时显示 `轴 项 = 值  PID:ON/OFF`；改动**下一帧即生效**（不用重启；丢靶预判/搜索不受影响）。

防跳项：同一按钮两次点击需间隔 ≥ `center_calib.debounce_ms`（默认 300ms）——触摸抖动产生的"重复按下"会被自动吞掉（**换按钮不受限**，不影响快速换键操作）；想更快可调小或设 0 禁用，仍嫌飘可调大。

排障：按键无反应 → 查 6P 触摸排线、看串口 `calib: touch ...` 行（触摸异常自动禁用，不影响追踪主程序）；
按 `▲` 绿点却往反方向走 → 把 `center_calib.touch_rotate` 从 -1 改 0/1/2/3 逐个试（坐标旋转）。

### 5.2 主机侧辅助（可选）

旧方式：把靶标对准画面中心，测激光点相对中心的像素偏移，手填 `config.json` 的
`center.laser_offset_x/y`（激光在右/下为正）。

## 六、主机侧自检（不接硬件）

```bash
python tools/test_servo_protocol.py  # V16 舵机：FSUS 帧/解析/控制纯函数/servo_send 闭环仿真（35 用例）
python tools/test_emm_protocol.py    # 协议帧对拍 + V8 读取命令/清堵转帧（24 用例）
python tools/test_coast_logic.py     # 丢靶预判状态机（10 用例）
python tools/test_circle_logic.py    # V10/V14 几何：画圈点/θ/面积/搜索/引脚/半径/测距激光（41 用例）
python tools/test_camera_init.py     # MaixCAM2 采集守门：camera.Camera/640x480/center/显示直显/旧 API（11 用例）
python tools/test_center_calib.py    # V7.7 触屏校准：按钮命中/触摸状态机/config 文本替换/8 键布局（37 用例）
python tools/test_pid_tune.py        # V7.9 PID+触屏调参：pid_step/参数步进/段内键保存/调参状态机（59 用例）
python tools/test_kf_filter.py       # V8 卡尔曼滤波：kf_step 收敛/噪声抑制/lead/限幅/reset + 接线守卫（18 用例）
python tools/test_attitude_loop.py   # V15 姿态：px/deg、倾角、att_step/outer/turn + 闭环仿真（45 用例）
```

九套均为 stdlib 独立脚本，从 `k230/main.py` AST 提取目标函数/常量后断言（不需要硬件、不需要 pytest），合计 **280 用例**。
改动协议层先跑 `test_servo_protocol.py` / `test_emm_protocol.py`；改动视觉/控制/配置面先跑其余几套；再上机。
（`test_servo_protocol.py` 的帧样本取自 `云台/FashionStar_协议完整帧表.md` 与串口实测，改帧构造必须先跑它。）

## 七、现场部署检查清单

- [ ] 两台驱动器均按 `接线图.md` 预置（校验 0x6B / 115200 / UART_FUN / 应答 Receive）
- [ ] K230 与两台驱动器电源**共地**
- [ ] 上电前：云台 pitch 已摆到行程中部（步进断电会下垂）
- [ ] 上电后 `actuator:` 行两轴均 `online`
- [ ] `config.json` 中 mode、input_size、labels、task 与模型匹配（YOLO 模式）
- [ ] 摄像头画面正常
- [ ] 先单轴 `test_mode` 自检（yaw 持续旋转 / pitch 往返±角度），再跑视觉追踪
- [ ] 方向核对：发正 dx → yaw 向减小偏差方向转；正 dy → pitch 同理
- [ ] 激光偏移已标定：触屏 `▲▼◀▶` 调绿点、满意后按 `S` 保存（或手填 `center.laser_offset_x/y`）
- [ ] 模式切换核对：触屏 `T/C` 按钮切换追踪/画圈（串口打印 `MODE->circle (touch)`）；若用跳线：`mode_pin.enabled=true` 且上电核对 `mode pin:` 电平打印
- [ ] 触屏调参核对：`P` 进调参页 → `项`/`+`/`-` 调整（串口打印 `tune: ...`、下一帧生效）；`S` 保存打印 `param: save OK saved 18 keys`，重启后参数保持
- [ ] 画圈核对：开机后看串口 `CIRCLE autocalib ready: k=...`（环干净时数秒内出现；未出现则查红环检测与 `calib_frames`）；切到画圈模式后应看到橙色合成圆（`OK [geo]`）或蓝色检测圆（`OK [ring]`）并绕环画圆；退化时屏上显示 `CIRCLE: no ring/k`；追踪模式应打印 `OK [rect]`
- [ ] 不摆靶开机 → yaw 匀速搜索；放靶 1~2 秒内停转进入追踪
- [ ] **V15 姿态**：上电看 `att:` 行确认 IMU 型号/`mag=False`，手转底座核对 `ATT th=`/`w=` 符号
- [ ] **V15 姿态**：`attitude.enabled=false` 时 OSD 应显示 `ATT θ d z 模式`、串口 1Hz 有 `ATT th=... err=...`；手转底座 θ 与肉眼转角误差 <1°；据此定 `yaw_sign`
- [ ] **V15 姿态**：开机串口应打印 `att cfg: on=.. ppd=.. tick=..` 指纹行（没有=板上不是 V15 代码）
- [ ] **V15 姿态**：开 `enabled=true` 且 `outer_gain=0` 时手转底座，画面靶标应纹丝不动；越纠越偏 → `yaw_sign` 取反
- [ ] **V15 姿态**：手转 ±90° / 来回摆 / 连续单向转，`err` 与靶心偏移 ≤0.5°（≈6px）；出现低频摆动先降 `rate_ki`
- [ ] 准备备用 SD 卡（预烧录系统+代码）

## 八、故障排查

| 问题 | 排查 |
|------|------|
| 检测不到矩形 | 降低 `rectangle.min_area`，放宽 `white_low`（CV 模式仅支持黑底白矩形）|
| 画面视野偏移（偏左/偏右）| 核查 `camera.csi_id`（须为 2）与采集提示 1280×720；**勿用 1280×960**（实测该模式视野偏左）|
| 画面横向被压缩（比例不对）| 输出须与 720 提示同为 16:9（640×360）；`set_framesize` 的 `crop=True` 为兜底（输出若被改成 4:3 会自动居中裁切而非拉伸）|
| 画面放大太多（视野窄）| 输出须为 640×360（与 720 提示同 16:9）＝全幅视野；显示大小由输出尺寸决定，改比例会改变视野 |
| 画面显小/黑边大 | 正常现象：显示为 1:1 帧直显（640×360 居中，四周黑边）。软件放大已被实机证伪（花屏/卡死），勿再引入 |
| 触屏校准按键无反应 | 查 6P 触摸排线；看串口 `calib: touch ...` 行（触摸异常会自动禁用，不影响追踪）；确认 `center_calib.enabled=true` |
| 触屏绿点移动方向不对 | `center_calib.touch_rotate` 从 -1 改 0/1/2/3 逐个试（触摸坐标旋转）|
| 误检太多 | 提高 `rectangle.min_area`，收窄 `min_aspect/max_aspect` |
| 帧率低 | 调大 `debug.gc_every`、关 `display.show_osd`，或将 `detection.detect_every` 设 2（隔帧检测）|
| 某轴电机不动 | 先看 `actuator:` 行是否 `online`；`(no resp)` → 按 `接线图.md` 第六节排障（共地/交叉/预置/回环）|
| 云台乱转/抖动 | 调死区 / 降低 K 参数 / 检查方向位（`reverse`）|
| pitch 电机不动/动作不对 | 与 yaw 同排查路径（接线图第六节）；确认 `[pitch_motor]` 参数与 `[motor]` 同构 |
| 电机被"卡住"后拒绝指令 | 堵转保护触发 → 检查机械卡阻，用上位机/命令解除堵转保护（0x0E 52）；或开 `clog_reset_on_boot=true` 上电自动恢复 |
| K230 画面卡 | 增大 `debug.gc_every`，降低检测分辨率 |
| YOLO模式报错 | 确认 kmodel 路径正确、KPU 库已安装 |
| YOLO不出框 | ① `input_size` ② `labels` 顺序 ③ `task` 配对 ④ 是否拍到训练过的物体 |
| 画圈不转圈（屏上 `CIRCLE: no ring/k`） | 环被激光自干扰（结构性）→ 走几何主轨：等 `CIRCLE autocalib ready` 或手填 `radius_ratio`（见 4.6）；可选开 `fallback_ms` 记忆兜底；查 `ring_min_diam_px/ring_max_diam_px` 与环的实际像素尺寸、红色阈值 |
| 画圈方向反了 | `circle.direction` 取反（1/-1）|
| 开机不搜索 | 确认 `search.enabled=true` 且开机后从未捕获过目标；已捕获过则永不搜索（设计如此）|
| 引脚切换不生效（用跳线时）| 核对 `mode_pin.gpio/active_level`；看启动 `mode pin:` 打印与实际跳线电平；悬空电平不可信，务必接 3.3V 或 GND |
| 触屏 `T/C` 不切换 | 若 `mode_pin.enabled=true`（跳线模式），`T/C` 会故意失效（以引脚为准）；默认模式由触屏切换（`mode_pin.enabled=false`）|
| PID 振荡/过冲 | 调小该轴 `pid_kp`（触屏调参页 `Kp`）；检查 `pid_ki` 是否过大；必要时 `PID` 项调 OFF 回原映射对比 |
| 开了 PID 但感觉没变 | 默认 `pid_kp`≈原映射斜率（0.28/0.14），行为接近属正常；用 Ki/Kd 或加大 Kp 拉开差异；确认轴别选错（调参页 `Y/P`）|
| 调参页 `S` 保存 FAIL | 看串口后缀：`keys not found` → config.json 缺 `pid_*` 新键（用仓库内最新 config.json）；`verify fail` → 写回后读回校验不过，检查文件是否被外部改坏 |
| UART报 tx not configured | 检查 config 中 `tx_pin`/`rx_pin` 与 FPIOA 映射 |
| **V15** 姿态没接管（OSD 灰字 `ATT .. idle`）| 串口看 `att:` 行：`固件无 maix.ext_dev.imu` → 固件不支持；`IMU 初始化失败` → 按 S11 查；`att cfg: on=N` → `[attitude].enabled` 还是 false 或触屏 `att` 项没打开 |
| **V15** 开姿态后云台越纠越偏/自己甩 | `yaw_sign` 配反（它已含 `motor.reverse`，不必再改 reverse）；发散保护会在 300ms 内自动关掉并打印 `att: DISABLED` |
| **V15** 姿态下低频来回摆动 | 降 `rate_ki`（延迟大时 0.6 以上会形成极限环，见 4.10 稳定边界表）；仍摆再降 `angle_kp` |
| **V15** 中心附近「嗒嗒」响/原地颤 | `deadband_dps` 调大（默认 3°/s）、`min_pulse_ms` 调大（默认 25ms）；或把 `min_rpm` 保持 1 别再调高 |
| **V15** 强扰动时视觉帧率掉到 ~30fps | 正常（帧间预算 6ms 让给内环）；嫌低就调小 `tick_budget_ms`，代价是抗扰余量变小 |
| **V15** 丢靶后不动也不搜 | `hold_ms` 还没到（默认 3000）；`search.turns>0` 且圈数已用完 → 不再重启（重启需重新锁定过目标）；`[search].enabled=false` 也会不搜 |
| **V15** `px_per_deg` 明显不对（跟随发散/迟滞）| 填 `[attitude].px_per_deg` 覆盖；自动反算依赖 `[distance].ref_area_px/ref_distance_mm` 与 `[circle].rect_w_cm` 的既有标定，标定错了它就跟着错 |

> 串口类问题的完整排障顺序（OLED 状态 → 共地 → 回环测试 → 引脚实测）见 `接线图.md` 第六节。

## 九、参考代码吸收纪要（V8）

V8 对「参考代码」做了完整对标分析（`参考代码\`：STM32H723 电赛云台 + Jetson 视觉；`官方代码\`：K230 霍夫圆例程），吸收情况如下：

**已吸收**：

| 借鉴项 | 参考出处 | 本项目落地 |
|---|---|---|
| 2 状态卡尔曼滤波（位置+速度） | robot_cmd.c VisionKalmanFilterInit | `kf_*` 键 + `kf_step` 纯函数（并用上参考中闲置的速度分量做 `kf_lead_s` 前馈），见 4.8 |
| 画圆"几何优先、不依赖检测" | robot_cmd.c 画圆模式（60mm 几何生成） | `circle.fallback_ms` 环记忆兜底（框中心+记忆半径续画），见 4.6 |
| 闭环驱动器读取能力 | X_V2.c 读取命令表 | `read_velocity`/`read_position`/`reset_clog` + 堵转监控，见 4.9 |
| 执行器加速档实义（提速第一性依据） | Emm42 说明书 Rev1.3 P36 公式 | V8.6：`acc` 默认 250 + config 注释公式化 + 追踪链路对照分析见第十节 |

**经论证不采纳**（防未来重复评估）：

| 候选 | 不采纳理由 |
|---|---|
| 透视校正（warpPerspective 全套） | 本项目的四边形对角线交点求中心已透视正确；K230 上额外算力换不来精度提升 |
| 距离自适应增益（1/D 映射） | 推导：`θ ≈ (dx_px / px_per_mm) / D` 且 `px_per_mm ∝ 1/D` → 像素误差→角度的增益天然与距离无关；仅在 mm 域工作才需要该补偿 |
| 双框嵌套检测（面积比 ≥0.7） | 当前靶面为「白心+黑边单框+红环」结构，双框策略无对应形态（若换靶纸再评估） |
| 30×30 闭运算修"激光打断黑边" | 大核形态学在 K230 上成本高；现有黑边验证已容忍单边断裂；现场出现实际丢检再以小核定实现 |
| 921600 双板串口协议（0xA5+float32） | 双板架构专用；本项目 K230 单板直控，无跨板链路 |

**硬件结论**：核心硬件无需新增；候选升级与理由见 `接线图.md` 第八章（读取命令表）与第九章（硬件升级建议）。

## 十、追踪链路对照分析（V8.6 对标参考代码）

对标对象：`参考代码\`（Jetson 视觉 + STM32H723 云台）。以下为 V8.6 的完整分析结论（软件侧优化已按此落地）。

### 10.1 参考的追踪链路（三层闭环）

| 层 | 机制 | 关键参数 | 出处 |
|---|---|---|---|
| 视觉 | 100fps 相机；透视校正到 mm 域；每帧发 `dx_mm/dy_mm/distance_mm`；4×4 闭运算修激光干扰 | 1.5px/mm | `视觉代码带注释.py` L1876/L1924/L2018 |
| 外环 | 200Hz：KF 滤波 → PID → 输出×距离自适应增益 → **累加到角度目标** | Kp 1.5~3 / Ki 3~4 / Kd 0.05~0.07 | `robot_cmd.c` L84/L158/L200 |
| 内环 | 1kHz：IMU 角度环+速度环（另有速度/电流前馈） | GM6020/DM 无刷直驱 | `gimbal.c` L74/L116 |
| 画圆 | **几何生成** 60mm 圆周目标点，PID 跟踪「误差↔圆周点」偏移（零检测） | radius=60mm | `robot_cmd.c` L191/L670 |

参考"快"的本质：**高频闭环（外 200Hz + 内 1kHz）+ 前馈**，指令到执行延迟 ~10-20ms；执行器为无刷直驱，不受步进转速斜坡限制。

### 10.2 本项目差距 → V8.6 对策

| 因子 | 参考 | 本项目（V8.6 前 → 后） | 对策 |
|---|---|---|---|
| **命令建立时间** | 电流前馈瞬时 | acc=200：每+1RPM 2.8ms（0→60RPM 需 168ms、换向±60 需 336ms）→ acc=250：0.3ms/RPM（0→90RPM 仅 27ms） | 默认 acc=250 |
| 速度上限 / 满速阈值 | 数千 RPM | 60/30RPM、200px → **90/45RPM、150px** | 参数包 |
| 控制频率 | 外200Hz+内1kHz | 单环 ~45fps | 结构差异（不换架构），用 acc/上限补偿 |
| 前馈 / 积分 | 速度+电流前馈；Ki 3~4 | `kf_lead_s` / PID 默认关 | 现场可选开启（见 4.8 / 4.3） |

### 10.3 Emm42 加减速公式（说明书 Rev1.3 P36 原文）

`t2 - t1 = (256 - acc) × 50μs`（每 +1RPM 的用时）；**acc=0 为特殊值：不使用曲线加减速，直接按设定速度运行**。常用对照：200≈2.8ms/RPM；240≈0.8；250≈0.3；255≈0.05。这是本次「追踪提速」的第一性依据（含换向：±60RPM 差速在 acc=250 下约 18ms 完成）。
