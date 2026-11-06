"""
MaixCAM2 物体追踪 — 双轴云台 (双步进)   [V11: 平台迁移 K230-CanMV → MaixCAM2 (MaixPy)]
================================
  左右(yaw)轴:  Emm42_V5.0 闭环步进驱动器  (UART2, 串口速度模式)
  上下(pitch)轴: Emm42_V5.0 闭环步进驱动器  (UART3, 串口速度模式)

═══════════════════════════════════════════════════════════════
  功能说明
═══════════════════════════════════════════════════════════════
  摄像头采集 → 物体检测 → 算目标中心与屏幕中心的偏移 → 驱动云台

  检测方式 (config.json → detection.mode):
    cv    : 矩形靶标检测(白心+黑边验证)   偏移 = 四边形中心 - 屏幕中心
    yolo  : (待移植 maix.nn, 当前自动回退 cv)

  执行器 (两轴都是 Emm42 闭环步进, 速度控制逻辑相同, 由 config.json 的 motor / pitch_motor 段配置):
    左右轴(yaw)  : 按 dx 做「速度」控制 —— |dx| 越大转速越快; 死区内停转;
                   V7.9 可切 PID(pid_step, 参数触屏可调);
                   丢靶预判: 目标从左右两侧丢失后, 朝最后方向续追一小段时间(可关, 见 coast_step)
    上下轴(pitch): 按 dy 做「速度」控制 —— |dy| 越大转速越快; 死区内停转(V7.9 同上可切 PID); 丢靶停转

  V8 可选增强(均默认关, 见 config.json 对应键):
    卡尔曼滤波(kf_*)            误差平滑 + 速度前馈(移植参考云台 2 状态 KF);
    堵转监控(monitor_ms)/清堵转(clog_reset_on_boot)  依赖 Emm42 读取命令(0x35/0x0E52)

  dx = 目标中心x - 屏幕中心x   (右偏为正)
  dy = 目标中心y - 屏幕中心y   (下偏为正)

═══════════════════════════════════════════════════════════════
  接线 (MaixCAM2 排针; 引脚名以丝印为准, 详见 skill/maixcan2引脚图.md / 官方引脚图)
═══════════════════════════════════════════════════════════════
  yaw 步进 (Emm42_V5.0, TTL 串口 UART2, 左侧 2x6P 排针):
      MaixCAM2 TX = B0(UART2_TX) → 驱动器① RXD
      MaixCAM2 RX = B1(UART2_RX) → 驱动器① TXD        (交叉接)
      共地; 驱动器① V+ 独立供电 7~32V(勿接 MaixCAM2 的 5V)
      驱动器侧需预置: 校验方式=固定0x6B, 波特率=115200, 串口ID=motor.addr, 运动模式=UART_FUN

  pitch 步进 (Emm42_V5.0, TTL 串口 UART3, 右侧 2x6P 排针):
      MaixCAM2 TX = B2(UART3_TX) → 驱动器② RXD
      MaixCAM2 RX = B3(UART3_RX) → 驱动器② TXD        (交叉接, 全双工)
      共地; 驱动器② V+ 独立供电 7~32V(勿接 MaixCAM2 的 5V)
      驱动器侧需预置: 校验方式=固定0x6B, 波特率=115200, 串口ID=pitch_motor.addr, 运动模式=UART_FUN

  模式引脚(可选, 默认关): A4(右侧排针, SPI1_SCK 丝印位) 跳线接 3V3=画圈, 否则=仅追踪

  !! UART0(A28/A29) 是系统日志串口, 勿占用; IO 均为 3.3V, 勿直连 5V。
  !! 一个 UART 控制器只能接一台驱动器: yaw 固定占 UART2、pitch 固定占 UART3, 不可混用。
     两者若配成同一个 port, 程序启动时会打印 WARN。

═══════════════════════════════════════════════════════════════
  文件结构 (自上而下)
═══════════════════════════════════════════════════════════════
   一  配置加载          从 config.json 读参数(跳过 _ 开头的注释键)
   二  运行参数          把配置铺平成模块级常量
   三  Emm 步进驱动      EmmStepper (速度模式 + 位置模式)          → yaw / pitch
   三点五 PID 速度控制   pid_step/pid_speed —— 误差→转速(可选切 PID, Kp/Ki/Kd 触屏可调)
   三点五·五 KF 误差滤波  kf_step/_kf_clear —— 2 状态卡尔曼(可选, 速度前馈)
   四  执行器初始化      gimbal_send(dx,dy,dist,status) —— 视觉与执行器之间的唯一接口
   五  CV 矩形检测       find_paper_box() 等
   六  YOLO 支持         _yolo_ok()  (V11 起待移植, 恒回退 cv)
   六点五 触屏交互        calib_* 校准(绿点/模式/保存) + param_* 调参页(PID/Kp/Ki/Kd/速度/死区…)
   七  CV 主循环         run_cv()
   八  YOLO 主循环       run_yolo()  (V11 起为占位提示)
   九  入口              test_mode 自检 → 按 mode 进对应主循环

  所有可调参数都在 config.json 内并附注释; 本文件不含硬编码阈值, 改完重启生效。
  V7.9: PID 开关/Kp/Ki/Kd 与两轴速度/死区/加速等参数可在触屏调参页实时调整, S 键写回 config。
  V8: 卡尔曼滤波 / 堵转监控均为可选增强(默认关), 键位与说明见 config.json。
  V9: 距离估算(面积法)+激光点映射(移植参考视觉v2.3, 见 config.json [distance]);
      aim_laser=true 时矩形追踪收敛点=激光落点(默认关, 仅显示距离/激光点)。
  V10: 画圈改纯几何 —— 圆心=矩形中心, 半径按靶纸固定尺寸(白框25.6×17 / 圆⌀11.3, 可配)
       推算; 删除红环检测(find_red_ring/[red_target]/渐进标定), 画圈不再受激光干扰。
  V11: 平台迁移 K230-CanMV → MaixCAM2(MaixPy): 采集 camera.Camera(BGR888 直出, cv2 零拷贝),
       显示 display.Display(FIT_CONTAIN 居中), 触摸 touchscreen.TouchScreen(坐标映射回图像系),
       串口 maix.uart+pinmap(yaw=UART2 B0/B1, pitch=UART3 B2/B3), GPIO maix.gpio, 退出 app.need_exit()。
  V13: 触屏 —— 每帧排空触摸事件缓冲(available/read0), 补回视觉掉帧时被压掉的"松手再按"沿;
       静止看门狗(stale_ms=800 无事件强制抬手)防固件丢松开导致状态卡高; read 异常不再永久禁用触摸。
       画圆 —— θ 增加到位等待超时(wait_max_ms=1000): 连续未到位超时后继续推进,
       脱机不接电机/远点偏心够不着时黄点不再钉死在圆上。
  V15: 姿态稳定(板载 IMU LSM6DSOW, 免接线) —— 云台底座快速旋转时不丢靶。
       结构: 视觉外环(改方位设定值 θ_ref) + 姿态内环(400~650Hz, 按陀螺把相机绝对
       姿态压在 θ_ref 上)。底座怎么转都被内环当场抵消, 不必等下一帧视觉。
       为什么这么绕: IMU 与相机同一刚体, 陀螺测到的是「底座转速 + 云台自身转速」之和,
       两者不可分离, 所以不用"测到扰动就反向补偿"的前馈, 而是闭环控制相机自身姿态。
       零点: 上电位置即 0 点; 靶心方位 = 首次锁定时「θ_imu + dx/px_per_deg」。
       px_per_deg 由 [distance]/[circle] 既有标定自动反算焦距, 免新增标定。
       丢靶: 先按 IMU 保持世界方位(hold_ms), 超时后单方向按圈搜索(IMU 计 360°)。
       发散保护: 角度误差超 fault_deg 持续 300ms → 自动关姿态、回退旧映射并告警。
       enabled=false 时行为与 V14 完全一致(走 [motor] 旧映射/PID/KF/coast)。
"""

import gc, math, os
import numpy as np
import cv2
from maix import camera, display, image, app, uart, pinmap, gpio, touchscreen, time

# ---- V15: 板载 IMU(MaixCAM2 = LSM6DSOW, IIC1/0x6B, 无磁力计) ----
# 固件里没有该模块时不抛错, 只把姿态功能整体禁用(旧逻辑照常跑)。
try:
    from maix.ext_dev import imu as _imu_mod
except Exception:
    _imu_mod = None

# ============================================================================
# 一、配置加载
# ============================================================================

def _tdiff(a, b):
    """毫秒差 a-b。真机实测 time.ticks_diff 的符号与 MicroPython 标准语义相反
    (b>a 时 diff(b,a) 返回负值, 见 V13 触摸防抖日志: now-act_t=+5906 而 diff 返回 -5906),
    导致防抖恒吞/看门狗永不触发/连发与θ超时永不满足。
    ticks_ms 为 64 位单调毫秒, 运行期内无回绕, 直接算术相减恒正确, 故弃用 ticks_diff。"""
    return a - b

CFG_PATH = None    # 实际命中的 config.json 路径(触屏校准「保存」回写用)

def _load_json(paths):
    """从候选路径加载JSON, 跳过_开头的注释键"""
    global CFG_PATH
    try:
        import ujson as j
    except ImportError:
        import json as j
    for p in paths:
        try:
            with open(p, "r") as f:
                raw = j.load(f)
            cfg = {}
            for k, v in raw.items():
                if k.startswith("_"):
                    continue
                if isinstance(v, dict):
                    cfg[k] = {sk: sv for sk, sv in v.items() if not sk.startswith("_")}
                else:
                    cfg[k] = v
            print("config: %s loaded" % p)
            CFG_PATH = p
            return cfg
        except Exception as e:
            print("config: 候选 %s 不可用 (%s)" % (p, e))
    print("config: using defaults")
    print("!!! 警告: 未找到 config.json -> 全部参数走代码默认值, 与你调好的配置不一致!")
    print("!!! 修法: 改用「运行项目」按钮(整个文件夹一起上传), 或把 config.json 上传到 /root/")
    return {}

def _get(cfg, key, default):
    return cfg.get(key, default) if cfg else default

_CFG_DIR = os.path.dirname(os.path.abspath(__file__))
# 优先级: 项目自带的(脚本同目录/上级) > 运行目录 > 持久目录 /root > SD 卡
#   「运行项目」会把整个文件夹传到 _CFG_DIR; 「运行当前文件」只传单个 .py, 此时靠 /root/ 里的副本兜底
CFG = _load_json([os.path.join(_CFG_DIR, "config.json"),
                  os.path.join(_CFG_DIR, "..", "config.json"),
                  "config.json",
                  "/root/config.json",
                  "/sd/config.json", "/sdcard/config.json"])
print("boot: 解析常量 ...")

# ============================================================================
# 二、运行参数 (全部来自 config.json)
# ============================================================================

# ---- 摄像头 (MaixCAM2: maix.camera.Camera, 直接出目标尺寸; 宽高比≠传感器16:9 时居中裁切) ----
# V12: 采集尺寸取屏幕分辨率 640x480 => FIT_CONTAIN 缩放比 1.0, 显示无黑边铺满全屏
SENSOR_W  = _get(_get(CFG, "camera", {}), "sensor_width", 640)
SENSOR_H  = _get(_get(CFG, "camera", {}), "sensor_height", 480)
FPS       = _get(_get(CFG, "camera", {}), "fps", 60)

# ---- 显示屏 (MaixCAM2 自带 640x480 触摸屏; disp.show 默认 FIT_CONTAIN 居中, 偏移键废弃) ----
SHOW_OSD  = _get(_get(CFG, "display", {}), "show_osd", True)

# ---- 检测模式 ----
MODE = _get(_get(CFG, "detection", {}), "mode", "cv")   # "cv" / "yolo"
DETECT_EVERY = _get(_get(CFG, "detection", {}), "detect_every", 1)

# ---- 调试输出节奏 ----
DBG = _get(CFG, "debug", {})
PRINT_EVERY = DBG.get("print_every", 60)   # 每 N 帧打印一次检测信息
GC_EVERY = DBG.get("gc_every", 30)         # 每 N 帧执行一次垃圾回收

# ---- 屏幕中心 (追踪目标点) ----
CENTER_CFG = _get(CFG, "center", {})
CENTER_X = CENTER_CFG.get("x", SENSOR_W // 2)
CENTER_Y = CENTER_CFG.get("y", SENSOR_H // 2)

# ---- CV 矩形检测阈值 ----
r = _get(CFG, "rectangle", {})
WHITE_LOW  = tuple(r.get("white_low",  [135, 135, 115]))
WHITE_HIGH = tuple(r.get("white_high", [255, 255, 255]))
BLACK_LOW  = tuple(r.get("black_low",  [0, 0, 0]))
BLACK_HIGH = tuple(r.get("black_high", [85, 85, 85]))
MIN_AREA   = r.get("min_area", 3500)
MAX_AREA   = r.get("max_area", 180000)
MIN_ASPECT = r.get("min_aspect", 1.05)
MAX_ASPECT = r.get("max_aspect", 2.80)
BORDER_EXPAND_X = r.get("border_expand_x", 16)
BORDER_EXPAND_Y = r.get("border_expand_y", 16)
BLACK_CHECK_STEP = r.get("black_check_step", 4)
MIN_BLACK_HITS   = r.get("min_black_hits", 3)
CENTER_WHITE_STEP = r.get("center_white_step", 3)
MAX_CENTER_JUMP = r.get("max_center_jump", 220)
MAX_AREA_RATIO  = r.get("max_area_ratio", 3)
MIN_RECT_FILL   = r.get("min_rect_fill", 45)
MAX_SIDE_RATIO  = r.get("max_side_ratio", 3)
APPROX_EPSILON  = r.get("approx_epsilon", 0.04)

# ---- 执行器配置 ----
# 左右(yaw)轴: [motor] 段, Emm42 步进-速度模式;  上下(pitch)轴: [pitch_motor] 段, Emm42 步进-速度模式(同逻辑)
mv = _get(CFG, "motor", {})                 # [motor] 段 → yaw
MOTOR_ENABLED = mv.get("enabled", False)
pm = _get(CFG, "pitch_motor", {})           # [pitch_motor] 段 → pitch
PITCH_ENABLED = pm.get("enabled", False)

# ---- V7.9 触屏可调运行时参数(初值=config; 触屏调参页 +/- 实时改, S 写回; 见六点五节) ----
#   这些是 gimbal_send 每帧读取的"活参数": 触屏改后下一帧即生效(不必改文件重启)。
#   pid_* 为 V7.9 新增的 PID 速度控制参数(enabled=false 时走原线性映射, 行为不变)。
M_MIN_RPM = mv.get("min_rpm", 1)        # yaw 最低转速
M_MAX_RPM = mv.get("max_rpm", 30)       # yaw 最高转速(也是 PID 输出限幅)
M_DX_FULL = mv.get("dx_full_px", 200)   # |dx| 达此值给满速(原映射)
M_DEADBAND = mv.get("deadband_px", 5)
M_ACC = mv.get("acc", 200)
M_PID_EN = mv.get("pid_enabled", False)
M_PID_KP = mv.get("pid_kp", 0.28)
M_PID_KI = mv.get("pid_ki", 0.0)
M_PID_KD = mv.get("pid_kd", 0.0)
P_MIN_RPM = pm.get("min_rpm", 1)        # pitch 最低转速
P_MAX_RPM = pm.get("max_rpm", 30)       # pitch 最高转速(也是 PID 输出限幅)
P_DY_FULL = pm.get("dy_full_px", 200)
P_DEADBAND = pm.get("deadband_px", 5)
P_ACC = pm.get("acc", 200)
P_PID_EN = pm.get("pid_enabled", False)
P_PID_KP = pm.get("pid_kp", 0.14)
P_PID_KI = pm.get("pid_ki", 0.0)
P_PID_KD = pm.get("pid_kd", 0.0)

# ---- V8 KF 误差滤波(初值=config; kf_enabled=false 时 gimbal_send 对误差原样直通, 行为不变) ----
#   2 状态卡尔曼(位置+速度)滤波视觉误差: 平滑检测抖动; kf_lead_s>0 时用速度估计做提前量(补帧延迟)。
#   移植自参考云台 robot_cmd.c:84-135 的 VisionKalmanFilterInit(2 维 KF, 观测位置)。
KF_Y_EN = mv.get("kf_enabled", False)      # yaw: KF 开关
KF_Y_QPOS = mv.get("kf_q_pos", 0.5)        # 过程噪声(位置)
KF_Y_QVEL = mv.get("kf_q_vel", 5.0)        # 过程噪声(速度)
KF_Y_R = mv.get("kf_r", 2.0)               # 观测噪声
KF_Y_LEAD = mv.get("kf_lead_s", 0.0)       # 速度前馈提前量(s); 0=关
KF_Y_LIM = mv.get("kf_err_limit", 150)     # 输出限幅(px)
KF_P_EN = pm.get("kf_enabled", False)      # pitch: 同上
KF_P_QPOS = pm.get("kf_q_pos", 0.5)
KF_P_QVEL = pm.get("kf_q_vel", 5.0)
KF_P_R = pm.get("kf_r", 2.0)
KF_P_LEAD = pm.get("kf_lead_s", 0.0)
KF_P_LIM = pm.get("kf_err_limit", 150)

# ---- V8 执行器监控/恢复(初值=config; 挖掘 Emm42 读取能力, 仅告警不动控制环) ----
M_MONITOR_MS = mv.get("monitor_ms", 0)              # yaw 堵转监控周期(ms); 0=关(默认)
P_MONITOR_MS = pm.get("monitor_ms", 0)              # pitch 同
M_CLOG_RESET = mv.get("clog_reset_on_boot", False)  # 上电发送清堵转(0x0E52); 默认关
P_CLOG_RESET = pm.get("clog_reset_on_boot", False)

# ---- V16 双舵机云台 (FashionStar FSUS 总线舵机, 单总线) ----
#   与步进路径互斥: servo.enabled=true 时不初始化 EmmStepper, 由 servo_send 接管 gimbal_send。
#   协议/帧构造见「三点二」; 控制律见「三点三」; 启动/状态机见「四」。
sv = _get(CFG, "servo", {})
SERVO_ENABLED = sv.get("enabled", False)
S_PORT = sv.get("port", 2)                 # UART 控制器编号(/dev/ttyS2)
S_BAUD = sv.get("baud", 115200)
S_TX = sv.get("tx_pin", "B0")              # MaixCAM2 B0 -> 1k -> 舵机信号
S_RX = sv.get("rx_pin", "B1")              # 直接并到信号线
S_YAW_ID = sv.get("yaw_id", 0)             # 默认 yaw=ID0 (可对调)
S_PITCH_ID = sv.get("pitch_id", 1)         # 默认 pitch=ID1
S_YAW_SIGN = -1.0 if sv.get("yaw_sign", 1) < 0 else 1.0
S_PITCH_SIGN = -1.0 if sv.get("pitch_sign", 1) < 0 else 1.0
S_YAW_MIN = sv.get("yaw_min", -150.0)      # 软件软限位(必须 <= 机械行程)
S_YAW_MAX = sv.get("yaw_max", 150.0)
S_PITCH_MIN = sv.get("pitch_min", -30.0)
S_PITCH_MAX = sv.get("pitch_max", 30.0)
S_YAW_KP = sv.get("yaw_kp", 0.18)          # °/s per px
S_YAW_KI = sv.get("yaw_ki", 0.08)
S_YAW_KD = sv.get("yaw_kd", 0.01)
S_YAW_KFF = sv.get("yaw_kff", 0.04)
S_PITCH_KP = sv.get("pitch_kp", 0.14)
S_PITCH_KI = sv.get("pitch_ki", 0.06)
S_PITCH_KD = sv.get("pitch_kd", 0.008)
S_PITCH_KFF = sv.get("pitch_kff", 0.03)
S_MAX_DPS = sv.get("max_dps", 150.0)       # 角速度上限(°/s)
S_MAX_STEP = sv.get("max_step_deg", 6.0)   # 每帧目标角最大增量(°)
S_BOOST_PX = sv.get("boost_px", 0.0)       # |误差|>该值直接满速; 160=640 宽的 1/4; 0=关
S_DEADBAND = sv.get("deadband_px", 3)      # 死区(px)
S_IZONE = sv.get("i_zone_px", 60)          # 积分分离阈值(px)
S_INTERVAL = sv.get("interval_ms", 35)     # 舵机走完一帧目标的周期(ms)
S_POWER = sv.get("power_mw", 0)            # 功率上限(mW); 0=不限
S_SYNC = sv.get("sync", True)              # true=cmd25 一帧同步两轴
S_HOLD_MS = sv.get("hold_ms", 1500)        # 丢靶保持(ms)
S_BOOT_CENTER = sv.get("boot_center", False)
S_CENTER_MS = sv.get("center_ms", 2000)
# 软件零点(不写舵机 EEPROM): 逻辑角 = 舵机原始多圈角 - 零点偏移。
#   zero_auto=true  -> 启动时把当前姿态记为逻辑 0°(偏移=读到的原始角), 断电不保持;
#   zero_auto=false -> 用固定的 yaw_zero_deg / pitch_zero_deg 作偏移。
# 注意 soft limits / search 范围 / boot_center 都工作在这套「逻辑角」坐标系里。
S_ZERO_AUTO = sv.get("zero_auto", False)
S_YAW_ZERO = sv.get("yaw_zero_deg", 0.0)
S_PITCH_ZERO = sv.get("pitch_zero_deg", 0.0)
S_SEARCH_EN = sv.get("search_enabled", True)
S_SEARCH_DPS = sv.get("search_dps", 25.0)
S_SEARCH_RANGE = sv.get("search_range_deg", 120.0)
S_SEARCH_DIR = sv.get("search_dir", 1)
S_SEARCH_MAX_MS = sv.get("search_max_ms", 0)
S_KF_EN = sv.get("kf_enabled", False)      # 首次上机建议 false; 稳定后再开
S_KF_QPOS = sv.get("kf_q_pos", 0.5)
S_KF_QVEL = sv.get("kf_q_vel", 5.0)
S_KF_R = sv.get("kf_r", 2.0)
S_KF_LEAD = sv.get("kf_lead_s", 0.0)
S_KF_LIM = sv.get("kf_err_limit", 150)
S_TEST_MODE = sv.get("test_mode", False)   # 不跑视觉, 只做 ping/点动自检
S_PID_EN = sv.get("pid_enabled", True)     # 触屏 PID 开关(关=纯 P, Ki/Kd/Kff 不计)

# KF 运行状态(每轴一个 7 元素列表 [x, v, p00, p01, p10, p11, t]; t=0 表首帧;
# 丢靶/搜索/死区时与 _pid_clear 同点调用 _kf_clear)
_kf_yaw = [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0]
_kf_pit = [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0]

# PID 运行状态(每轴一套: i=积分累计 px·s / e=上次误差 / t=上次时刻; 丢靶/死区时清零)
_pid_yaw_i = 0.0
_pid_yaw_e = 0.0
_pid_yaw_t = 0
_pid_pit_i = 0.0
_pid_pit_e = 0.0
_pid_pit_t = 0

# ---- V9 距离估算 + 激光点映射(移植参考视觉v2.3: 面积法测距/距离线性映射激光点) ----
dc = _get(CFG, "distance", {})
DIST_EN = dc.get("enabled", True)                 # 总开关; false=完全不计算(行为与 V8 一致)
DIST_REF_MM = dc.get("ref_distance_mm", 1500)     # 标定距离: 靶标(A4纸)放置处(mm)
DIST_REF_AREA = dc.get("ref_area_px", 8412)       # 标定距离下检测到的靶标像素面积
DIST_MIN = dc.get("dist_min_mm", 500)             # 映射下限(近于此按最近处理)
DIST_MAX = dc.get("dist_max_mm", 1500)            # 映射上限(远于此按最远处理)
LASER_X = dc.get("laser_x", 322)                  # 激光点X固定坐标(激光器安装横向偏移)
LASER_Y_NEAR = dc.get("laser_y_near", 240)        # 最近距离的激光点Y(=画面中心; V12 起画面为 640x480)
LASER_Y_FAR = dc.get("laser_y_far", 250)          # 最远距离的激光点Y(=中心+10)
LASER_AIM = dc.get("aim_laser", False)            # true=矩形追踪收敛点=激光落点; false=仅显示(默认)

# ---- V7 激光偏移(激光点相对画面中心; 与光轴同轴则 0; 触屏校准运行时调整它) ----
LOX = CENTER_CFG.get("laser_offset_x", 0)
LOY = CENTER_CFG.get("laser_offset_y", 0)

# ---- V7.7 触屏中心校准(绿点=追踪收敛点=CENTER+LOX/LOY; 「保存」写回 [center] 的 laser_offset) ----
cb = _get(CFG, "center_calib", {})
CALIB_ENABLED   = cb.get("enabled", True)
CALIB_STEP      = cb.get("step_px", 4)          # 每按一次/每连发一拍移动的像素
CALIB_REPEAT    = cb.get("repeat_ms", 120)      # 按住连发间隔(ms)
CALIB_DEBOUNCE  = cb.get("debounce_ms", 300)    # 两次点击最小间隔(防触摸抖动"一下跳两项"; 0=禁用)
CALIB_DOT_R     = cb.get("dot_radius", 2)       # 绿点半径(px; 小点口径)
CALIB_SHOW_DOT  = cb.get("show_dot", True)      # 启动时绿点+按钮是否显示
TOUCH_STALE_MS  = cb.get("stale_ms", 800)       # 触摸看门狗: 按住但长时间无事件(固件丢松开)判定抬手(ms; 0=关)
CALIB_XMAX = SENSOR_W // 2                      # 校准范围上限(防呆)
CALIB_YMAX = SENSOR_H // 2

# ---- V7 上电搜索 ----
srch = _get(CFG, "search", {})
SEARCH_EN = srch.get("enabled", True)
SEARCH_RPM = srch.get("rpm", 12)
SEARCH_DIR = srch.get("direction", 1)
SEARCH_MAX_MS = srch.get("max_ms", 0)
# V15: 姿态模式下的搜索 = 单方向按圈扫(用 IMU 积分计 360°, 比写死匀速时间准)
SEARCH_TURNS = srch.get("turns", 0)      # 连续搜索圈数; 0=不限圈数(一直找), N=转满 N 圈停
SEARCH_DPS = srch.get("dps", 25)         # 扫掠的世界角速度(°/s); 内环据此产生恒定世界转速

# ---- V7 画圈(V10: 纯几何 — 圆心=矩形中心, 半径按固定尺寸比例推算, 不检测红环) ----
cc = _get(CFG, "circle", {})
CIRC_SPEED = cc.get("speed_deg_s", 60)
CIRC_DIR = cc.get("direction", 1)
CIRC_PAUSE = cc.get("pause_px", 120)
CIRC_WAIT_MAX = cc.get("wait_max_ms", 1000)  # 连续未到位多久后不再等待继续转(ms; 0=旧行为永久等)
RADIUS_BIAS_CM = cc.get("radius_bias_cm", 0.0)  # 目标半径修正(cm): 光斑系统性偏内/偏外时微调(正=加大)
PAPER_W_CM = cc.get("paper_w_cm", 29.7)       # A4 白纸宽(cm)
PAPER_H_CM = cc.get("paper_h_cm", 21.0)       # A4 白纸高(cm)
TAPE_W_CM = cc.get("tape_w_cm", 2.0)          # 白纸外沿黑胶带宽度(cm)
RECT_W_CM = cc.get("rect_w_cm", 0) or (PAPER_W_CM - 2.0 * TAPE_W_CM)  # 白色可见矩形宽(胶带内沿); 0=按 paper-2×tape 推导
RECT_H_CM = cc.get("rect_h_cm", 0) or (PAPER_H_CM - 2.0 * TAPE_W_CM)  # 白色可见矩形高(胶带内沿); 0=按 paper-2×tape 推导
CIRCLE_DIA_CM = cc.get("circle_dia_cm", 11.3) # 中心圆直径(cm), 与白矩形同心

# ---- V7 模式引脚(V11 起为 MaixCAM2 引脚名, 默认 A4: 被供电=画圈) ----
mp = _get(CFG, "mode_pin", {})
PIN_ENABLED = mp.get("enabled", False)   # 缺省关(与文档一致: 默认触屏 T/C 切换, 引脚为可选备用)
PIN_GPIO = mp.get("gpio", "A4")          # MaixCAM2 丝印引脚名(字符串), 如 "A4"
PIN_ACTIVE = mp.get("active_level", 1)
PIN_DEBOUNCE = mp.get("debounce_n", 5)

# ---- V15 姿态稳定(板载 IMU, 与相机同一刚体) ----
#   内环 400~650Hz 把相机绝对姿态压在视觉给的方位设定值 θ_ref 上; 视觉只改设定值。
#   enabled=false(默认) 时行为与 V14 完全一致: [motor] 旧映射 / PID / KF / coast 照旧。
at = _get(CFG, "attitude", {})
ATT_EN_CFG = at.get("enabled", False)      # 配置文件里的初值(运行时可能被触屏/发散保护改写)
ATT_YAW_SIGN = -1 if at.get("yaw_sign", 1) < 0 else 1   # 姿态方向; 越转越偏就取反(-1)
ATT_TICK_US = at.get("tick_period_us", 1500)   # 内环拍间隔(us); 1500≈650Hz
ATT_BUDGET_MS = at.get("tick_budget_ms", 6)    # 每帧最多给内环多少 ms(其余留给视觉)
ATT_GYRO_DPS = at.get("gyro_dps", 2000)        # 陀螺量程(°/s); 手转可能超 500, 默认给足
ATT_ANGLE_KP = at.get("angle_kp", 12.0)        # 角度环 P: (度/秒)/度; 12 ≈ 时间常数 83ms
ATT_RATE_KI = at.get("rate_ki", 0.55)          # 速率环积分/扰动观测器增益(0~1; 越大越快越易振)
ATT_DEADBAND_DPS = at.get("deadband_dps", 3.0) # 速率死区(°/s): 低于此不开步进(防低速"嗒嗒")
ATT_MIN_PULSE_MS = at.get("min_pulse_ms", 25)  # 最小脉宽(ms): 抑制零附近通断抖动
ATT_MIN_RPM = at.get("min_rpm", 1)             # 姿态模式最低转速(1RPM=6°/s; 别用 [motor] 的 5)
ATT_PX_PER_DEG_CFG = at.get("px_per_deg", 0)   # 像素/度; 0=由 [distance] 标定自动反算
ATT_OUTER_GAIN = at.get("outer_gain", 0.8)     # 视觉外环: 每帧把测量误差的多少比例计进 θ_ref
ATT_OUTER_LEAD = at.get("outer_lead_s", 0.0)   # 外环速度前馈提前量(s); 0=关
ATT_HOLD_MS = at.get("hold_ms", 3000)          # 丢靶后保持世界方位的时长(ms); 超时转搜索
ATT_FAULT_DEG = at.get("fault_deg", 15.0)      # 发散保护: 角度误差超此值持续 300ms → 降级
ATT_ACTIVE_DPS = at.get("active_dps", 15.0)    # 扰动超过此角速度才把帧间预算全给内环
ATT_TILT_ALPHA = at.get("tilt_alpha", 0.98)    # 倾角互补滤波系数(陀螺权重)
ATT_ACC = at.get("acc", 255)                   # 姿态模式 yaw 的加减速档(255≈0.05ms/RPM, 建立快)

# ============================================================================
# 三、Emm42_V5.0 闭环步进驱动 —— yaw / pitch(均为速度模式)
#    协议来自 云台/Emm_V5.0步进闭环驱动说明书Rev1.3.pdf
# ============================================================================
#  帧格式: [地址][功能码][数据...][校验]
#    - 默认校验方式 = 固定值, 末字节恒为 0x6B;  串口 115200 8N1
#    - 地址: 1~255, 0=广播
#  本程序用到的命令:
#    使能       addr F3 AB <en> <sync> 6B     en: 01=使能 00=失能
#    速度模式   addr F6 <dir> <spdH> <spdL> <acc> <sync> 6B          —— yaw 轴用
#               dir: 00=CCW 01=CW;  spd=0~3000(RPM);  acc=0~255(越大加速越快)
#    位置模式   addr FD <dir> <spdH> <spdL> <acc> <p3><p2><p1><p0> <abs> <sync> 6B   —— 备用(未接入)
#               16细分下 3200 脉冲/圈; abs: 00=相对 01=绝对; 目标为32位(负数用补码)
#    位置清零   addr 0A 6D 6B                  (把当前物理位置定义为 0° —— pitch 上电用)
#    触发回零   addr 9A <mode> <sync> 6B      (0=单圈就近 1=单圈方向 2=多圈碰撞 3=限位开关; 预留)
#    立即停止   addr FE 98 <sync> 6B
#    读总线电压 addr 24 6B  →  回 addr 24 <VH> <VL> 6B  (mV, 兼作在线探测)
#  !! 注意: 本协议所有多字节字段均为"高字节在前"(大端), 与 Modbus 相反。
#     依据说明书标注: 速度 05 DC=0x05DC=1500RPM; 脉冲 00 00 7D 00=32000;
#     电压 5C 6A=23658mV
# ============================================================================

EMM_CHECKSUM = 0x6B   # 校验方式=固定值时, 末字节恒为该值


class EmmStepper:
    """Emm42_V5.0 闭环步进驱动器 (TTL 串口; 速度模式 + 位置模式)

    接线(port/tx_pin/rx_pin/addr 由 config.json 各段提供, 如 [motor]/[pitch_motor]):
      MaixCAM2 TX = tx_pin(引脚名, 如 "B0") -> 驱动器 RXD
      MaixCAM2 RX = rx_pin(引脚名, 如 "B1") -> 驱动器 TXD      (交叉接)
      GND 必须共地;  驱动器 V+ 独立供电 7~32V (勿接 MaixCAM2 5V)

    驱动器侧需预先设好(上位机/OLED 菜单): 校验=固定0x6B, 波特率=115200,
    串口ID=addr, 运动模式=UART_FUN

    速度模式: 追踪时由 |偏差| 算出目标转速, 由偏差符号定方向    (yaw / pitch 现用)
    位置模式: 下发绝对目标位置(脉冲), 配合 clear_position() 定义 0°;
              当前未接入控制流(备用, 见 move/home; 脉冲换算依赖细分, 16细分=3200/圈)。
    """

    def __init__(self, uart_id=2, baud=115200, tx_pin="B0", rx_pin="B1",
                 addr=1, acc=200, timeout_ms=50):
        # MaixCAM2: 先 pinmap 把引脚映射到 UARTx_TX/RX, 再按设备节点 /dev/ttyS{uart_id} 开串口
        # 逐调用标记: 启动若卡在构造里, 最后打印的那行就是卡住的调用
        print("    emm%d> pinmap %s -> UART%d_TX" % (uart_id, tx_pin, uart_id))
        print("      ret = %r" % (pinmap.set_pin_function(tx_pin, "UART%d_TX" % uart_id),))
        print("    emm%d> pinmap %s -> UART%d_RX" % (uart_id, rx_pin, uart_id))
        print("      ret = %r" % (pinmap.set_pin_function(rx_pin, "UART%d_RX" % uart_id),))
        print("    emm%d> open /dev/ttyS%d" % (uart_id, uart_id))
        self.uart = uart.UART("/dev/ttyS%d" % uart_id, baud)
        print("    emm%d> open ok" % uart_id)
        self.addr = addr & 0xFF
        self.acc = acc        # 默认加减速档位(set_speed 可覆盖)
        self.tmo = timeout_ms
        self.last_rpm = 0     # V8: 最近一次速度指令(带符号); 供堵转监控对照(见 run_cv)

    # ---- 底层收发 ----
    def _send(self, func, data=b""):
        """组帧并发送: [addr][func][data...][0x6B]"""
        f = bytearray(3 + len(data))
        f[0] = self.addr
        f[1] = func
        f[2:2 + len(data)] = data
        f[-1] = EMM_CHECKSUM
        self.uart.write(bytes(f))
        return f

    def _recv(self, n, timeout_ms=None):
        """读满 n 字节, 超时返回 None"""
        t = self.tmo if timeout_ms is None else timeout_ms
        buf = bytearray()
        t0 = time.ticks_ms()
        while len(buf) < n:
            if _tdiff(time.ticks_ms(), t0) > t:
                return None
            chunk = self.uart.read(n - len(buf), 0)   # 非阻塞读剩余缺口; 无数据返回 b''
            if chunk:
                buf += chunk
        return buf

    # ---- 高层动作 ----
    def enable(self, en=True, sync=False):
        """使能/失能电机 (F3)"""
        try:
            self.uart.read(-1, 0)               # 清空上次残留
        except Exception:
            pass
        self._send(0xF3, bytes([0xAB,
                                0x01 if en else 0x00,
                                0x01 if sync else 0x00]))

    def stop(self, sync=False):
        """立即停止 (FE 98)"""
        self.last_rpm = 0    # V8: 停止即清零命令记录(堵转监控参考)
        self._send(0xFE, bytes([0x98, 0x01 if sync else 0x00]))

    def set_speed(self, direction, rpm, acc=None, sync=False):
        """速度模式 (F6): direction>0=CW / <0=CCW; rpm=0~3000"""
        s = int(rpm)
        if s < 0:
            s = 0
        elif s > 3000:
            s = 3000
        a = self.acc if acc is None else acc
        if a < 0:
            a = 0
        elif a > 255:
            a = 255
        self.last_rpm = s if direction > 0 else -s   # V8: 记录命令值(堵转监控参考)
        self._send(0xF6, bytes([0x01 if direction > 0 else 0x00,
                                (s >> 8) & 0xFF, s & 0xFF,    # 速度: 高字节在前
                                a & 0xFF,
                                0x01 if sync else 0x00]))

    def move(self, pulses, rpm, acc=None, absolute=True, sync=False):
        """位置模式 (FD): 目标位置=脉冲数; absolute=True 时按绝对位置下发。

        16细分下 3200 脉冲/圈; 负目标用 32 位补码编码; dir 位按目标正负给
        (>=0 → CCW 0x01)。绝对模式下 dir 位的实际作用以实机验证为准。
        """
        p = int(pulses) & 0xFFFFFFFF
        s = int(rpm)
        if s < 0:
            s = 0
        elif s > 3000:
            s = 3000
        a = self.acc if acc is None else acc
        if a < 0:
            a = 0
        elif a > 255:
            a = 255
        self._send(0xFD, bytes([
            0x01 if pulses >= 0 else 0x00,
            (s >> 8) & 0xFF, s & 0xFF,
            a & 0xFF,
            (p >> 24) & 0xFF, (p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF,
            0x01 if absolute else 0x00,
            0x01 if sync else 0x00]))

    def clear_position(self):
        """位置清零 (0A 6D): 把当前位置角度/误差/脉冲数全部清零。

        pitch 轴用它把「上电位置」定义为 0°(要求上电前云台位于行程中部)。
        """
        self._send(0x0A, bytes([0x6D]))

    def home(self, mode=0, sync=False):
        """触发回零 (9A): mode 0=单圈就近 1=单圈方向 2=多圈无限位碰撞 3=多圈限位开关。

        本项目当前未接入常规流程(预留); 回零参数需先在驱动器菜单里设好。
        """
        self._send(0x9A, bytes([mode & 0xFF, 0x01 if sync else 0x00]))

    def read_bus_voltage(self):
        """读总线电压 (24) → mV; 无应答/校验错返回 None (兼作在线探测)"""
        try:
            self.uart.read(-1, 0)
        except Exception:
            pass
        self._send(0x24)
        r = self._recv(5)
        if r is None or r[0] != self.addr or r[1] != 0x24 or r[4] != EMM_CHECKSUM:
            return None
        return (r[2] << 8) | r[3]                            # 高字节在前

    # ---- V8 读取类命令(转速/位置) + 清堵转 ----------------------------------
    # 命令码来源: 张大头官方例程 X_V2.c 读取命令表; 帧式与读电压 0x24 同构(已实机验证)。
    # !! 响应字节段为「预期格式」: 首次上机先 raw_probe(0x35)/raw_probe(0x36) 捕获
    #    真实响应核对, 再把实际样本补进 tools/test_emm_protocol.py 固化。

    def read_velocity(self, timeout_ms=None):
        """读电机实时转速 (0x35) → 带符号 RPM; 无应答/校验错返回 None。

        请求: [addr][0x35][0x6B]
        预期响应 6B: [addr][35][sign][v_hi][v_lo][0x6B]  (sign: 0=正/1=负)。
        """
        try:
            self.uart.read(-1, 0)
        except Exception:
            pass
        self._send(0x35)
        r = self._recv(6, timeout_ms)
        if r is None or r[0] != self.addr or r[1] != 0x35 or r[5] != EMM_CHECKSUM:
            return None
        v = (r[3] << 8) | r[4]                                # 高字节在前
        return -v if r[2] else v

    def read_position(self, timeout_ms=None):
        """读电机实时位置 (0x36) → 带符号脉冲数; 无应答/校验错返回 None。

        请求: [addr][0x36][0x6B]
        预期响应 8B: [addr][36][sign][pos×4 高字节在前][0x6B]。
        (位置语义——单圈/多圈/上电零点——以实机核对为准; 供软限位评估用。)
        """
        try:
            self.uart.read(-1, 0)
        except Exception:
            pass
        self._send(0x36)
        r = self._recv(8, timeout_ms)
        if r is None or r[0] != self.addr or r[1] != 0x36 or r[7] != EMM_CHECKSUM:
            return None
        p = (r[3] << 24) | (r[4] << 16) | (r[5] << 8) | r[6]  # 高字节在前
        return -p if r[2] else p

    def reset_clog(self):
        """清除堵转保护 (0x0E 0x52): 撞限位/卡死触发堵转保护后恢复电机。

        帧: [addr][0x0E][0x52][0x6B] (命令来源: 第三方驱动库 ZDT_EmmV5_Driver;
        首次使用以实机行为核对——发一次后看驱动器是否解除堵转)。
        """
        self._send(0x0E, bytes([0x52]))

    def raw_probe(self, func=0x24, timeout_ms=300):
        """诊断用: 发一帧后把 RX 上收到的原始字节全部 dump 出来(不做校验/解析)。
        返回 (sent, recv); recv 为空表示一个字节都没收到。
        - recv == sent        → 短接了 TX/RX 的回环, MaixCAM2 收发链路正常
        - recv 非空且 != sent → 收到了, 但格式不符(多半驱动器校验方式不是固定0x6B)
        - recv 为空           → MaixCAM2 没发出去 / 驱动器没上电 / 驱动器串口关了
        """
        try:
            self.uart.read(-1, 0)
        except Exception:
            pass
        sent = self._send(func)
        buf = bytearray()
        t0 = time.ticks_ms()
        while _tdiff(time.ticks_ms(), t0) < timeout_ms:
            chunk = self.uart.read(-1, 0)     # 非阻塞读缓冲区全部; 无数据返回 b''
            if chunk:
                buf += chunk
                t0 = time.ticks_ms()      # 收到就续命, 直到线路静默
        return bytes(sent), bytes(buf)


# ============================================================================
# 三点二、FashionStar FSUS 总线舵机 —— 协议帧/解析(纯函数) + 薄驱动类
#   帧: 12 4C [cmd] [len] [content...] [cs];  响应: 05 1C ...;  cs=Σ&0xFF; 小端; 0.1°
#   参考: 云台/FashionStar_协议完整帧表.md; 云台/cpp-sdk/.../sendSyncCommand
#   设计: 帧构造/解析全部做成纯函数(主机可测); 类只负责 UART 收发, 不含控制逻辑。
# ============================================================================

FS_ANGLE_MIN = -368640.0      # cmd13 协议范围(±368640°)
FS_ANGLE_MAX = 368640.0


def fs_u16(v):
    """uint16 → 2 字节小端"""
    v = int(v) & 0xFFFF
    return bytes((v & 0xFF, (v >> 8) & 0xFF))


def fs_u32(v):
    """uint32 → 4 字节小端"""
    v = int(v) & 0xFFFFFFFF
    return bytes((v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF))


def fs_i32(v):
    """int32 → 4 字节小端(负数用补码)"""
    return fs_u32(int(v) & 0xFFFFFFFF)


def fs_i32_at(b, off):
    """从字节串 off 处读 int32 小端"""
    v = b[off] | (b[off + 1] << 8) | (b[off + 2] << 16) | (b[off + 3] << 24)
    if v >= 0x80000000:
        v -= 0x100000000
    return v


def fs_i16_at(b, off):
    """从字节串 off 处读 int16 小端"""
    v = b[off] | (b[off + 1] << 8)
    if v >= 0x8000:
        v -= 0x10000
    return v


def fs_frame(cmd, content=b""):
    """组帧: 12 4C cmd len content cs"""
    n = len(content) & 0xFF
    body = bytes((0x12, 0x4C, cmd & 0xFF, n)) + bytes(content)
    return body + bytes(((sum(body)) & 0xFF,))


def fs_ping(sid):
    """cmd01 Ping: 12 4C 01 01 sid cs"""
    return fs_frame(0x01, bytes((sid & 0xFF,)))


def fs_set_mturn(sid, angle_deg, interval_ms=0, power_mw=0):
    """cmd13 多圈绝对角: 12 4C 0D 0B sid angle(i32) interval(u32) power(u16) cs"""
    a = int(round(max(FS_ANGLE_MIN, min(FS_ANGLE_MAX, angle_deg)) * 10.0))
    return fs_frame(0x0D, bytes((sid & 0xFF,)) + fs_i32(a)
                    + fs_u32(interval_ms) + fs_u16(power_mw))


def fs_set_single(sid, angle_deg, interval_ms=0, power_mw=0):
    """cmd08 单圈绝对角: 12 4C 08 07 sid angle(i16) interval(u16) power(u16) cs"""
    a = int(round(max(-180.0, min(180.0, angle_deg)) * 10.0))
    return fs_frame(0x08, bytes((sid & 0xFF,)) + fs_u16(a & 0xFFFF)
                    + fs_u16(interval_ms) + fs_u16(power_mw))


def fs_sync_mturn(items, interval_ms=0, power_mw=0):
    """cmd25 同步多圈帧: 一帧驱动多个舵机(子命令 13, 每舵机 11B)。

    items = [(sid, angle_deg), ...]; 两轴时整帧 30 字节, 115200 下约 2.6ms。
    """
    n = len(items)
    c = bytearray((0x0D, 0x0B, n))
    for sid, ang in items:
        a = int(round(max(FS_ANGLE_MIN, min(FS_ANGLE_MAX, ang)) * 10.0))
        c += bytes((sid & 0xFF,))
        c += fs_i32(a)
        c += fs_u32(interval_ms)
        c += fs_u16(power_mw)
    return fs_frame(0x19, bytes(c))


def fs_stop(sid, mode=1, power_mw=0):
    """cmd18 停止: mode 0=卸力 1=锁力 2=阻尼; 线上 mode|0x10"""
    return fs_frame(0x18, bytes((sid & 0xFF, (int(mode) | 0x10) & 0xFF))
                    + fs_u16(power_mw))


def fs_query_mturn(sid):
    """cmd16 读多圈角: 12 4C 10 01 sid cs"""
    return fs_frame(0x10, bytes((sid & 0xFF,)))


def fs_query_single(sid):
    """cmd10 读单圈角: 12 4C 0A 01 sid cs"""
    return fs_frame(0x0A, bytes((sid & 0xFF,)))


def fs_parse(buf):
    """解析响应帧: 返回 (cmd, content) 或 None(头/长度/校验错)"""
    if not buf or len(buf) < 6:
        return None
    b = bytes(buf)
    if b[0] != 0x05 or b[1] != 0x1C:
        return None
    cmd = b[2]
    n = b[3]
    if len(b) < 5 + n:
        return None
    content = b[4:4 + n]
    cs = (0x05 + 0x1C + cmd + n + sum(content)) & 0xFF
    if cs != b[4 + n]:
        return None
    return cmd, content


def fs_angle_mturn(content):
    """cmd16 响应的 content[1:5] → 多圈角(度); content 长度可能是 7, 只取前 5"""
    return fs_i32_at(content, 1) * 0.1


def fs_angle_single(content):
    """cmd10 响应的 content[1:3] → 单圈角(度)"""
    return fs_i16_at(content, 1) * 0.1


class FashionStarBus:
    """一条舵机总线 = 一个 UART。

    平台: **MaixCAM2 / MaixPy v4** —— `maix.uart.UART("/dev/ttySx", baud)` +
    `maix.pinmap.set_pin_function()`；不是庐山派 K230 的 `machine.UART` + `FPIOA`。
    K230 版仅作协议参考(skill/FASHIONSTAR-SERVO-K230-SKILL.md)，不要直接拷。
    """

    def __init__(self, uart_id=2, baud=115200, tx_pin="B0", rx_pin="B1",
                 timeout_ms=60):
        print("    fs%d> pinmap %s -> UART%d_TX" % (uart_id, tx_pin, uart_id))
        print("      ret = %r" % (pinmap.set_pin_function(tx_pin, "UART%d_TX" % uart_id),))
        print("    fs%d> pinmap %s -> UART%d_RX" % (uart_id, rx_pin, uart_id))
        print("      ret = %r" % (pinmap.set_pin_function(rx_pin, "UART%d_RX" % uart_id),))
        print("    fs%d> open /dev/ttyS%d" % (uart_id, uart_id))
        self.uart = uart.UART("/dev/ttyS%d" % uart_id, baud)
        print("    fs%d> open ok" % uart_id)
        self.tmo = timeout_ms

    def send(self, frame):
        """发一帧(写指令不回包); 发前清残留, 避免上一帧响应被当成本帧"""
        try:
            self.uart.read(-1, 0)
        except Exception:
            pass
        self.uart.write(frame)
        return frame

    def _recv(self, n, timeout_ms=None):
        """读满 n 字节; 超时返回 None"""
        t = self.tmo if timeout_ms is None else timeout_ms
        buf = bytearray()
        t0 = time.ticks_ms()
        while len(buf) < n:
            if _tdiff(time.ticks_ms(), t0) > t:
                return None
            chunk = self.uart.read(n - len(buf), 0)
            if chunk:
                buf += chunk
        return bytes(buf)

    def query(self, frame, timeout_ms=None):
        """发一帧并读响应; 返回 (cmd, content) 或 None。仅 boot/低频监控用。"""
        self.send(frame)
        hdr = self._recv(4, timeout_ms)
        if hdr is None or hdr[0] != 0x05 or hdr[1] != 0x1C:
            return None
        n = hdr[3]
        body = self._recv(n + 1, timeout_ms)
        if body is None:
            return None
        content = body[:n]
        cs = (0x05 + 0x1C + hdr[2] + n + sum(content)) & 0xFF
        if cs != body[n]:
            return None
        return hdr[2], content


class FashionStarServo:
    """单个 FSUS 舵机(挂在 FashionStarBus 上, 靠 ID 区分)"""

    def __init__(self, bus, sid):
        self.bus = bus
        self.id = sid & 0xFF
        self.angle = 0.0

    def ping(self):
        r = self.bus.query(fs_ping(self.id), 80)
        return bool(r and r[0] == 0x01 and r[1] and r[1][0] == self.id)

    def read_mturn(self):
        """读多圈角(度); 无响应/校验错返回 None"""
        r = self.bus.query(fs_query_mturn(self.id), 80)
        if not r or r[0] != 0x10 or len(r[1]) < 5:
            return None
        self.angle = fs_angle_mturn(r[1])
        return self.angle

    def read_single(self):
        """读单圈角(度); 无响应/校验错返回 None"""
        r = self.bus.query(fs_query_single(self.id), 80)
        if not r or r[0] != 0x0A or len(r[1]) < 3:
            return None
        self.angle = fs_angle_single(r[1])
        return self.angle

    def set_mturn(self, angle_deg, interval_ms=0, power_mw=0):
        self.angle = angle_deg
        self.bus.send(fs_set_mturn(self.id, angle_deg, interval_ms, power_mw))

    def set_single(self, angle_deg, interval_ms=0, power_mw=0):
        self.angle = angle_deg
        self.bus.send(fs_set_single(self.id, angle_deg, interval_ms, power_mw))

    def stop(self, mode=1, power_mw=0):
        self.bus.send(fs_stop(self.id, mode, power_mw))


# ============================================================================
# 三点三、舵机外环控制(纯函数, 主机可测) —— 增量式 PID + 前馈
#   参考云台 robot_cmd.c: 视觉误差 → KF → PID → 角度增量(不是 RPM);
#   舵机内部位置环负责快速跟踪, 外环只把「目标角」按 PID 输出累加。
# ============================================================================

def servo_pid(e, e_prev, i_acc, dt, kp, ki, kd, kff, e_vel,
              vmax, deadband, izone):
    """单轴增量式 PID(输出角速度 °/s)。返回 (u, i_new, e)。

    e       : 像素误差(带符号);  |e|<=deadband 时输出 0 并清积分
    e_prev  : 上次误差(无 KF 速度时用于差分)
    i_acc   : 积分累计(px·s);  |e|>=izone 时不积分(积分分离)
    e_vel   : KF 速度(px/s); None=用 (e-e_prev)/dt 差分
    kff     : 速度前馈增益(deg/px); 用 e_vel 直接补目标运动
    vmax    : 输出限幅(°/s); 饱和且误差同向时冻结积分(抗饱和)
    """
    if abs(e) <= deadband:
        return 0.0, 0.0, e
    if abs(e) < izone:
        i_new = i_acc + e * dt
    else:
        i_new = i_acc
    if e_vel is None:
        d = (e - e_prev) / dt if dt > 0 else 0.0
        ff = 0.0
    else:
        d = e_vel
        ff = e_vel
    u = kp * e + ki * i_new + kd * d + kff * ff
    if u > vmax:
        u = vmax
        if e > 0:
            i_new = i_acc
    elif u < -vmax:
        u = -vmax
        if e < 0:
            i_new = i_acc
    return u, i_new, e


def servo_step_target(target, u, dt, lo, hi, max_step):
    """把角速度 u 积分成新目标角; 返回 (new_target, saturated)。

    max_step: 每帧最大增量(°), 防一帧大跳/长帧放大; 到软限位也置 saturated,
    调用方据此冻结积分(抗饱和)。
    """
    step = u * dt
    sat = False
    if step > max_step:
        step = max_step
        sat = True
    elif step < -max_step:
        step = -max_step
        sat = True
    nt = target + step
    if nt > hi:
        nt = hi
        sat = True
    elif nt < lo:
        nt = lo
        sat = True
    return nt, sat


def servo_boost(u, e, boost_px, vmax):
    """大误差助推: |e| 超过 boost_px 时直接给 ±vmax, 不再按 Kp 线性爬。

    现场要求: 目标中心偏出「1/4 屏宽」就以最大速度追(640 宽 → 160px),
    靠近中心后交回 PID, 避免满速冲过死区。
    boost_px<=0 关闭(纯 PID)。
    """
    if boost_px > 0.0:
        if e > boost_px:
            return vmax
        if e < -boost_px:
            return -vmax
    return u


def servo_search_step(target, direction, dps, dt, lo, hi):
    """搜索扫掠: 目标角以 dps 在 [lo,hi] 往返; 返回 (new_target, new_direction)。

    目标在带外(例如追踪到 140° 后丢靶, 搜索带只有 ±120°)时先按原速回带内,
    不做一步 clamp 跳变(否则会命令一个 20° 的瞬跳)。
    """
    step = dps * dt
    if target >= hi:
        direction = -1
    elif target <= lo:
        direction = 1
    nt = target + (step if direction >= 0 else -step)
    if direction > 0 and nt > hi:
        nt = hi
        direction = -1
    elif direction < 0 and nt < lo:
        nt = lo
        direction = 1
    return nt, direction


# ============================================================================
# 三点五、PID 速度控制(纯函数, 供 gimbal_send 使用; 主机可测)
#   速度式 PID: 位置误差(px) → 转速(RPM)。M_PID_EN/P_PID_EN=true 时启用,
#   否则 gimbal_send 走原「线性映射+死区」。Kp/Ki/Kd 触屏可调, 改后下一帧即生效。
# ============================================================================

def pid_step(err, now_ms, i_acc, e_prev, t_prev, kp, ki, kd, lim):
    """一阶 PID 步进(速度式输出, RPM)。返回 (u, i_acc, e_prev)。

    err    : 位置误差(px, 带符号)
    now_ms : 当前时刻 ticks_ms();  t_prev: 上次时刻(0=首帧)
    i_acc  : 积分累计(px·s);       e_prev: 上次误差
    lim    : 输出限幅(正数, RPM)
    首帧(t_prev=0): dt 用 0.033s 兜底、微分为 0; 长帧 dt 截到 0.5s。
    抗饱和: 输出饱和且误差仍指向饱和方向时冻结积分(不累积)。
    """
    dt = 0.033
    d = 0.0
    if t_prev:
        dt = _tdiff(now_ms, t_prev) / 1000.0
        if dt <= 0.0:
            dt = 0.033
        elif dt > 0.5:
            dt = 0.5
        d = (err - e_prev) / dt
    i_new = i_acc + err * dt
    u = kp * err + ki * i_new + kd * d
    if u > lim:
        u = lim
        if err > 0:
            i_new = i_acc
    elif u < -lim:
        u = -lim
        if err < 0:
            i_new = i_acc
    return u, i_new, err

def pid_speed(u, min_rpm):
    """PID 输出 → (方向 1/-1/0, 绝对转速); |u|<min_rpm 且非零时抬到 min_rpm(保步进低速扭矩)"""
    if u > 0:
        return 1, (u if u >= min_rpm else min_rpm)
    if u < 0:
        r = -u
        return -1, (r if r >= min_rpm else min_rpm)
    return 0, 0

def _pid_clear(axis):
    """清空某轴(0=yaw / 1=pitch)的 PID 运行状态(丢靶/搜索/死区停转时调用)"""
    global _pid_yaw_i, _pid_yaw_e, _pid_yaw_t, _pid_pit_i, _pid_pit_e, _pid_pit_t
    if axis == 0:
        _pid_yaw_i = 0.0
        _pid_yaw_e = 0.0
        _pid_yaw_t = 0
    else:
        _pid_pit_i = 0.0
        _pid_pit_e = 0.0
        _pid_pit_t = 0


# ============================================================================
# 三点五·五、V8 KF 误差滤波(纯函数, 供 gimbal_send 使用; 主机可测)
#   2 状态卡尔曼(位置+速度, 观测位置): 平滑视觉误差抖动; kf_lead_s>0 时把速度
#   估计当「提前量」前馈出去(补偿检测/显示帧延迟)。移植自参考云台 robot_cmd.c
#   的 VisionKalmanFilterInit(2 维 KF), 并把它算了没用的速度分量用上。
#   kf_enabled=false(默认)时 gimbal_send 不调用本函数, 行为与旧版完全一致。
# ============================================================================

def kf_step(z, now_ms, st, q_pos, q_vel, r, lead_s, lim):
    """2 状态 KF 单步(观测位置)。st=[x, v, p00, p01, p10, p11, t] 就地更新; 返回输出(px)。

    z           : 观测(像素误差, 带符号)
    now_ms      : 当前时刻 time.ticks_ms();  st: 状态/协方差/上次时刻
    q_pos/q_vel : 过程噪声(位置/速度);  r: 观测噪声;  lim: 输出限幅(正数)
    lead_s      : 速度前馈提前量(s), 0=纯滤波输出(lead 不写入状态)

    首帧(t=0): 用观测初始化(x=z, v=0, P=diag(5,5) —— 对齐参考 P_init);
    dt<=0 兜底 0.033s、dt>0.5s 截断(与 pid_step 同约定)。
    """
    if st[6] == 0:                       # 首帧: 观测初始化
        st[0] = float(z)
        st[1] = 0.0
        st[2] = 5.0
        st[3] = 0.0
        st[4] = 0.0
        st[5] = 5.0
        st[6] = now_ms
    else:
        dt = _tdiff(now_ms, st[6]) / 1000.0
        st[6] = now_ms
        if dt <= 0.0:
            dt = 0.033
        elif dt > 0.5:
            dt = 0.5
        # 预测: x' = x + v*dt;  P' = F·P·Fᵀ + Q·dt
        x = st[0] + st[1] * dt
        p00 = st[2] + dt * (st[3] + st[4]) + q_pos * dt
        p01 = st[3] + dt * st[5]
        p10 = st[4] + dt * st[5]
        p11 = st[5] + q_vel * dt
        # 更新: S = P00' + r;  K = P'·Hᵀ/S;  x += K·(z - x')
        s = p00 + r
        k0 = p00 / s
        k1 = p10 / s
        y = z - x
        st[0] = x + k0 * y
        st[1] = st[1] + k1 * y
        st[2] = (1.0 - k0) * p00
        st[3] = (1.0 - k0) * p01
        st[4] = p10 - k1 * p00
        st[5] = p11 - k1 * p01
    xo = st[0] + st[1] * lead_s          # 速度前馈(提前量)
    if xo > lim:
        xo = lim
    elif xo < -lim:
        xo = -lim
    return xo

def _kf_clear(axis):
    """清空某轴(0=yaw / 1=pitch)的 KF 状态(丢靶/搜索/死区停转时调用, 与 _pid_clear 同点)"""
    global _kf_yaw, _kf_pit
    if axis == 0:
        _kf_yaw = [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0]
    else:
        _kf_pit = [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0]


# ============================================================================
# 三点七、V15 姿态解算与内环(纯函数, 主机可测)
#   tools/test_attitude_loop.py 抓取本节函数做主机侧**闭环仿真**(与 kf_step 同模式)。
#
#   物理前提: 板载 IMU 与相机在同一刚体上 → 陀螺测到的是「相机绝对角速度」
#     ω_cam = ω_base + m_motor
#   底座扰动与云台自身转动在传感器上不可分离, 所以不能用前馈猜扰动, 只能闭环控制
#   相机自身的绝对姿态:
#     ω_base_hat = ω_cam - k_g·rpm_prev       (扰动观测器: 反推底座转速)
#     ω_des      = angle_kp·(θ_ref - θ_imu)   (角度环: 把姿态压回设定值)
#     rpm_new    = (k_g·rpm_prev + rate_ki·(ω_des - ω_cam)) / k_g
#   视觉外环(att_outer)只改 θ_ref, 从不直接给转速。
# ============================================================================

ATT_RPM_TO_DPS = 6.0    # 1 RPM = 360°/60s = 6°/s (42 步进直驱; 若加了减速比改这里)


def att_px_per_deg(f_px):
    """焦距(px) → 像素/度(纯函数)。小角近似 θ ≈ dx/f_px (弧度)。"""
    if f_px <= 0:
        return 0.0
    return f_px * math.pi / 180.0


def att_focal_px(area_px, aspect, rect_w_mm, dist_mm):
    """由「靶标像素面积 + 已知物理尺寸 + 标定距离」反算焦距(px), 免新增标定(纯函数)。

    宽高比 aspect=RECT_W/RECT_H → 宽像素 = sqrt(area·aspect);
    f = 宽像素 × 距离 / 物理宽。任一参数不合理 → 0(调用方走默认值)。
    """
    if area_px <= 0 or aspect <= 0 or rect_w_mm <= 0 or dist_mm <= 0:
        return 0.0
    return math.sqrt(area_px * aspect) * dist_mm / rect_w_mm


def att_tilt(ax, ay, az, gx, gy, roll_prev, pitch_prev, alpha, dt_s):
    """互补滤波求横滚/俯仰角(度, 纯函数): 陀螺短时准, 加速度长时不漂。

    alpha≈0.98: 陀螺占大头(响应快), 加速度缓慢拉回(消除积分漂移)。
    dt_s<=0(首帧/异常帧) 时直接用加速度角, 不做陀螺预测。
    """
    a_roll = math.degrees(math.atan2(ay, az))
    a_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    if dt_s <= 0:
        return a_roll, a_pitch
    p_roll = roll_prev + gx * dt_s
    p_pitch = pitch_prev + gy * dt_s
    return (alpha * p_roll + (1.0 - alpha) * a_roll,
            alpha * p_pitch + (1.0 - alpha) * a_pitch)


def att_yaw_rate(gx, gy, gz, roll, pitch):
    """机体角速度 → 绕世界竖轴(yaw)的角速度(度/秒, 纯函数)。

    世界 z 轴在机体系的分量 = (-sinθ, cosθ·sinφ, cosθ·cosφ) (θ=pitch, φ=roll);
    与机体系角速度点乘即得绕世界竖轴的转速。水平安装(φ=θ=0) 时退化为 gz。
    相机随 pitch 轴抬头/低头时, 不做这个投影会让 yaw 转速被系统性低估。
    """
    r = math.radians(roll)
    p = math.radians(pitch)
    return (-gx * math.sin(p)
            + gy * math.cos(p) * math.sin(r)
            + gz * math.cos(p) * math.cos(r))


def att_step(theta, theta_ref, w_cam, rpm_prev, angle_kp, rate_ki,
             deadband_dps, min_rpm, max_rpm, fault_deg):
    """内环一拍(纯函数): 相机绝对角速度闭环 → 有符号整数 RPM。

    返回 (rpm, err_deg, fault):
      rpm     : 带符号整数(+/− 绕世界竖轴正/反方向), 0=停
      err_deg : 角度误差 θ_ref-θ_imu(度), 供 OSD 与发散保护
      fault   : |err_deg| > fault_deg (调用方累计时长后决定降级)
    增益口径: angle_kp 单位 (度/秒)/度; rate_ki ∈(0,1] 是速率环积分/观测器增益。
    """
    err = theta_ref - theta
    w_des = angle_kp * err
    m_prev = ATT_RPM_TO_DPS * rpm_prev
    m_new = m_prev + rate_ki * (w_des - w_cam)
    fault = abs(err) > fault_deg
    if abs(m_new) < deadband_dps:
        return 0, err, fault
    rpm = int(abs(m_new) / ATT_RPM_TO_DPS + 0.5)
    if rpm < min_rpm:
        rpm = min_rpm
    elif rpm > max_rpm:
        rpm = max_rpm
    return (rpm if m_new > 0 else -rpm), err, fault


def att_gate(rpm_cmd, rpm_prev, since_ms, min_pulse_ms):
    """最小脉宽门控(纯函数): 抑制零附近的通断抖动(步进低速会"嗒嗒"响)。

    仅当「本次要停(0) 而上一条非零命令还没跑满 min_pulse_ms」时沿用上一条;
    换向不受限制 —— 抗扰需要能立刻反向。返回最终应下发的 RPM。
    """
    if rpm_cmd == 0 and rpm_prev != 0 and min_pulse_ms > 0 and since_ms < min_pulse_ms:
        return rpm_prev
    return rpm_cmd


def att_outer(dx, px_per_deg, gain, deadband_px, d_prev, dt_s, lead_s):
    """视觉外环一拍(纯函数): 像素偏差 → 方位设定值 θ_ref 的增量(度)。

    返回 (delta_deg, d_now):
      delta = gain·(dx/ppd) + 前馈提前量(lead_s>0 时用误差变化率做预测, 限幅 ±5°)
      |dx| <= deadband_px → 0(中心附近不修正, 避免抖动)
    """
    d = float(dx)
    if abs(d) <= deadband_px or px_per_deg <= 0:
        return 0.0, d
    delta = gain * (d / px_per_deg)
    if lead_s > 0 and dt_s > 0:
        lead = gain * ((d - d_prev) / px_per_deg) * (lead_s / dt_s)
        if lead > 5.0:
            lead = 5.0
        elif lead < -5.0:
            lead = -5.0
        delta += lead
    return delta, d


def att_turn_step(theta_imu, theta_prev, theta_ref, direction, dps, dt_s,
                  swept_deg, turns):
    """单方向按圈搜索一拍(纯函数)。

    返回 (theta_ref_new, swept_new, done):
      θ_ref 按 direction·dps·dt 推进 → 内环据此产生**恒定世界角速度**的扫掠;
      swept 用「相机世界角的实际变化」累计 —— 底座同时也在转时, 真实扫过的才作数;
      turns>0 且 |swept| >= 360·turns → done(停转); turns<=0 = 不限圈数(一直找)。
    """
    if dt_s <= 0:
        return theta_ref, swept_deg, False
    ref = theta_ref + direction * dps * dt_s
    swept = swept_deg + direction * (theta_imu - theta_prev)
    return ref, swept, (turns > 0 and abs(swept) >= 360.0 * turns)


# ---- px/度 在模块加载时定值: 0=由 [distance]+[circle] 既有标定自动反算(免新增标定) ----
_att_ppd = ATT_PX_PER_DEG_CFG
if _att_ppd <= 0:
    _att_f_px = att_focal_px(DIST_REF_AREA,
                             (RECT_W_CM / RECT_H_CM) if RECT_H_CM > 0 else 0.0,
                             RECT_W_CM * 10.0, DIST_REF_MM)
    if _att_f_px > 0:
        _att_ppd = att_px_per_deg(_att_f_px)
    else:
        _att_ppd = 11.5
        print("att: px_per_deg 自动反算失败(查 [distance]/[circle] 标定) -> 暂用 11.5")


# ============================================================================
# 三点九、V15 姿态运行时(硬件读写 / 状态机 / 下发)
#   帧层(gimbal_send)只写 _att_mode 与 _att_ref; 内环(_att_tick)跑在帧间。
# ============================================================================

_imu = None              # IMU 实例(None=未初始化/失败/固件无该模块)
ATT_EN = False           # 运行时总开关(启动时按 ATT_EN_CFG 与 IMU 是否就绪定)
_att_tus = 0             # 上一拍时刻(us); 0=首拍
_att_th = 0.0            # θ_imu: 相机累计方位角(度; 上电位置=0, 不取模, 滑环可无限转)
_att_prev_th = 0.0       # 上一拍 θ_imu(搜索按圈计量用)
_att_ref = 0.0           # θ_ref: 方位设定值(即"靶心方位"), 视觉外环增量修正
_att_rpm = 0             # 上一拍下发的 RPM(带符号; 同时充当积分器状态)
_att_w = 0.0             # 最近一次相机绝对角速度(°/s)
_att_err = 0.0           # 最近一次角度误差(°)
_att_roll = 0.0
_att_pitch = 0.0
_att_mode = "idle"       # idle(不动) / lock(视觉锁定) / hold(丢靶保持) / search(按圈扫)
_att_hold_t = 0          # 进入 hold 的时刻(ms)
_att_swept = 0.0         # 搜索已扫过的世界角度(度)
_att_sdone = False       # 搜索圈数用完 → 不再重启(与旧 search.max_ms 的"超时后不再重启"同义)
_att_pulse_t = 0         # 上一条非零转速的下发时刻(ms; 最小脉宽门控)
_att_fault_t = 0         # 发散超限的起始时刻(ms); 0=当前正常
_att_d_prev = 0.0        # 上一帧 dx(外环 lead 用)
_att_d_t = 0             # 上一帧时刻(ms)
_att_cmd_sign = 1        # RPM 符号 → set_speed 方向(启动时 = M_REV · ATT_YAW_SIGN)
_att_drive = False       # 内环是否真的掌控电机; false=只观测 θ/ω 不发指令(P2 只读模式)

try:
    _ticks_us = time.ticks_us
except Exception:
    def _ticks_us():
        return time.ticks_ms() * 1000


def _tus():
    """微秒级单调时刻(固件无 ticks_us 时退化为 ms×1000)。

    内环 dt 必须用 us: 2.5ms 的拍长若只按 ms 量化, 每拍积分误差可达 20%,
    200°/s 时一帧累积起来就是 0.3° 量级的假漂移(吃掉 0.5° 预算)。
    """
    return _ticks_us()


def _att_reset(mode="idle"):
    """复位姿态零点与状态: 上电位置 = 0°; θ_ref 归零(靶心方位由首帧视觉累加得出)"""
    global _att_th, _att_ref, _att_prev_th, _att_rpm, _att_err, _att_w
    global _att_mode, _att_swept, _att_sdone, _att_hold_t, _att_fault_t, _att_tus
    global _att_d_prev, _att_d_t
    _att_th = 0.0
    _att_prev_th = 0.0
    _att_ref = 0.0
    _att_rpm = 0
    _att_err = 0.0
    _att_w = 0.0
    _att_swept = 0.0
    _att_sdone = False
    _att_hold_t = 0
    _att_fault_t = 0
    _att_tus = 0
    _att_d_prev = 0.0
    _att_d_t = time.ticks_ms()
    _att_mode = mode


def _att_busy():
    """是否要把帧间预算全给内环(在扫掠/保持, 或底座正在被扰动)"""
    if not _att_drive:
        return False
    if _att_mode == "search" or _att_mode == "hold":
        return True
    return abs(_att_w) > ATT_ACTIVE_DPS


def _att_tick():
    """内环一拍(硬件): 读 IMU → 姿态 → att_step/att_gate → 下发 RPM(仅变化时)。

    由 _att_spin 在视觉帧间反复调用; 只在 RPM 变化时写 UART(量化天然限流)。
    """
    global _att_th, _att_prev_th, _att_ref, _att_rpm, _att_w, _att_err, _att_roll
    global _att_pitch, _att_tus, _att_mode, _att_hold_t, _att_swept, _att_sdone
    global _att_pulse_t, _att_fault_t
    try:
        d = _imu.read()          # [ax, ay, az, gx, gy, gz, temp] (g / 度每秒 / 度)
    except Exception:
        return
    now_us = _tus()
    dt_us = now_us - _att_tus if _att_tus else 0
    if dt_us <= 0 or dt_us > 200000:      # 首拍/异常长拍: 按一个标称拍长兜底
        dt_us = ATT_TICK_US
    _att_tus = now_us
    dt_s = dt_us / 1000000.0

    ax = d[0]; ay = d[1]; az = d[2]; gx = d[3]; gy = d[4]; gz = d[5]
    _att_roll, _att_pitch = att_tilt(ax, ay, az, gx, gy, _att_roll, _att_pitch,
                                     ATT_TILT_ALPHA, dt_s)
    w = att_yaw_rate(gx, gy, gz, _att_roll, _att_pitch)
    _att_w = w
    _att_prev_th = _att_th
    _att_th += w * dt_s

    # 到这里 θ/ω 已经是最新的(OSD 可读); 只有真正接管电机时才跑状态机与内环。
    if not _att_drive:
        return

    # ---- 状态机里与"速率"相关的部分在这里推进(帧层只写模式) ----
    if _att_mode == "search":
        if _att_sdone:
            if _yaw:
                _yaw.stop()
            return
        _att_ref, _att_swept, _done = att_turn_step(
            _att_th, _att_prev_th, _att_ref, SEARCH_DIR, SEARCH_DPS, dt_s,
            _att_swept, SEARCH_TURNS)
        if _done:
            _att_sdone = True
            _att_mode = "idle"
            if _yaw:
                _yaw.stop()
            print("SEARCH done (%.0f°, %d turn, no target)"
                  % (_att_swept, SEARCH_TURNS))
    elif _att_mode == "hold":
        if ATT_HOLD_MS > 0 and _tdiff(time.ticks_ms(), _att_hold_t) > ATT_HOLD_MS:
            _att_mode = "search"
            _att_swept = 0.0
            print("HOLD timeout (%dms) -> SEARCH" % ATT_HOLD_MS)

    # ---- 内环 ----
    rpm, err, fault = att_step(_att_th, _att_ref, w, _att_rpm, ATT_ANGLE_KP,
                               ATT_RATE_KI, ATT_DEADBAND_DPS, ATT_MIN_RPM,
                               M_MAX_RPM, ATT_FAULT_DEG)
    _att_err = err
    rpm = att_gate(rpm, _att_rpm, _tdiff(time.ticks_ms(), _att_pulse_t),
                   ATT_MIN_PULSE_MS)
    if rpm != _att_rpm:
        if rpm == 0:
            _yaw.stop()
        else:
            _yaw.set_speed(_att_cmd_sign * (1 if rpm > 0 else -1), abs(rpm))
        _att_rpm = rpm
        _att_pulse_t = time.ticks_ms()

    # ---- 发散保护: 误差长时间超限 = 符号配反/增益不合理, 立刻退回旧映射 ----
    if fault:
        if _att_fault_t == 0:
            _att_fault_t = time.ticks_ms()
            print("WARN: att err=%+.1f° > %.1f° (yaw_sign 或增益配反?)"
                  % (err, ATT_FAULT_DEG))
        elif _tdiff(time.ticks_ms(), _att_fault_t) > 300:
            _att_disable("角度误差持续超限 (err=%+.1f°)" % err)
    else:
        _att_fault_t = 0


def _att_spin_run(budget_ms):
    """视觉帧间跑内环: 最多占用 budget_ms; 静止且在锁定态时只跑 1 拍。

    放在 cam.read() **之前**调用 —— 那一刻相机正在缓冲下一帧, 这段时间本来就在等,
    所以内环不额外吃掉视觉帧率。强扰动/扫掠/保持时才把预算用满(视觉帧率会降)。
    """
    if not _att_busy():
        _att_tick()
        return 1
    t_end = time.ticks_ms() + budget_ms
    n = 0
    while not app.need_exit():
        _att_tick()
        n += 1
        if _tdiff(time.ticks_ms(), t_end) >= 0 or n > 200:
            break
        time.sleep_us(ATT_TICK_US)
    return n


def _att_spin_off(budget_ms):
    """姿态关闭时的内环(空操作) —— 与 _att_spin_run 同签名, 便于整段回退"""
    return 0


def _att_bind(on):
    """启用/停用姿态接管 —— 顺带把「该走哪套实现」重绑一次, 热路径上不留分支。

    on=True 需要 _imu 就绪; 不就绪时保持旧映射并返回 False(不抛错)。
    触屏调参页的 att 开关与发散保护都走这里。
    """
    global ATT_EN, _trk_yaw, _lost_yaw, _search_yaw, _att_spin, _att_rpm, _att_drive
    if on and (_imu is None or _yaw is None):
        print("att: 无可用 IMU 或 yaw 轴未启用 -> 无法启用姿态接管")
        return False
    if on:
        ATT_EN = True
        _att_drive = True
        _trk_yaw = _trk_yaw_att
        _lost_yaw = _lost_yaw_att
        _search_yaw = _search_yaw_att
        _att_spin = _att_spin_run
        if _yaw:
            _yaw.acc = ATT_ACC          # 姿态模式用快建立档(acc 越大建立越快)
        _att_reset("idle")
        print("att: ON (px_per_deg=%.2f tick=%dus hold=%dms max_rpm=%d)"
              % (_att_ppd, ATT_TICK_US, ATT_HOLD_MS, M_MAX_RPM))
    else:
        ATT_EN = False
        _att_drive = False
        _trk_yaw = _trk_yaw_legacy
        _lost_yaw = _lost_yaw_legacy
        _search_yaw = _search_yaw_legacy
        # IMU 在就绪时仍保持观测(OSD 看 θ/ω), 只是不再驱动电机
        _att_spin = _att_spin_run if _imu is not None else _att_spin_off
        _att_rpm = 0
        if _yaw:
            _yaw.acc = M_ACC
            _yaw.stop()
        print("att: OFF (回退 [motor] 旧映射)")
    return True


def _att_disable(reason):
    """发散保护/异常降级: 关姿态并回退旧映射(只降级一次, 不反复打印)"""
    if not ATT_EN:
        return
    print("att: DISABLED -> %s" % reason)
    _att_bind(False)


_att_spin = _att_spin_off   # 启动时由 _att_bind 按 IMU 就绪情况重新绑定


# ============================================================================
# 四、云台执行器初始化 + gimbal_send
#    gimbal_send(dx, dy, dist, status) 是「视觉」与「执行器」之间唯一的接口:
#      dx → 左右轴(yaw, 步进)  : 速度模式 —— |dx| 决定转速, dx 符号决定方向
#      dy → 上下轴(pitch, 步进): 速度模式 —— |dy| 决定转速, dy 符号决定方向(与 yaw 相同逻辑)
#    status: "track"=追踪中 / "lost"=丢靶
#    "lost" 时: yaw 支持「丢靶预判」—— 目标从左右两侧丢失后朝最后方向续转一段
#    (无目标追踪, 见 coast_step 与 config [motor].coast_* 键; 默认关, coast_test=true 可单独自检, 先于 test_mode 执行)
#    pitch 丢靶即停。
#
#    !! 本段整体包在 try 里: 任何一条轴初始化失败都会打印
#       "!!! actuator init failed: ..." 并把 gimbal_send 降级成空操作(两轴都不动)。
#       所以上电后第一眼就看这行日志, 确认两条轴都正常。
# ============================================================================

_yaw = None        # yaw 轴: EmmStepper 实例(速度模式)
_pitch = None      # pitch 轴: EmmStepper 实例(速度模式)
_mode_pin = None   # 模式引脚(默认 A4): 被供电=画圈; maix.gpio.GPIO 实例(未启用/初始化失败=None)
_gimbal_ok = False # gimbal_send 是否已就绪(执行器初始化成功)——自检脚本用它防"静默假通过"

# ---- V16 双舵机路径状态(servo.enabled=true 时使用; 步进路径不受影响) ----
_sbus = None           # FashionStarBus(单总线)
_syaw = None           # FashionStarServo(yaw)
_spitch = None         # FashionStarServo(pitch)
_s_ok_y = False        # yaw 舵机在线(且已读到角度)
_s_ok_p = False        # pitch 舵机在线
_s_yaw_target = 0.0    # 目标角(°), 绝对多圈
_s_pitch_target = 0.0
_s_yaw_i = 0.0         # PID 积分(px·s)
_s_pitch_i = 0.0
_s_yaw_e = 0.0         # 上次误差(px)
_s_pitch_e = 0.0
_s_last_ms = 0         # 上次 servo_send 时刻
_s_mode = "idle"       # idle / track / hold / search
_s_hold_t = 0          # 进入 hold 的时刻
_s_search_t = 0        # 进入 search 的时刻
_s_search_dir = 1      # 当前扫掠方向(+1/-1)
_s_search_done = False # 搜索关闭/超时后不再重启(防每帧重复打印)
_s_kf_yaw = [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0]   # 2 状态 KF(与 V8 同结构)
_s_kf_pit = [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0]
_s_first = True        # 首帧/重捕获: 抑制 D 项初始 kick
_s_off_y = 0.0         # 软件零点偏移(°): 下发角 = 逻辑目标 + _s_off_y; 由 _servo_init 设定
_s_off_p = 0.0

# 丢靶预判(coast)状态 —— 由 gimbal_send 维护(见 coast_step)
_last_dx = 0           # 最后一次追踪的 yaw 偏差(带符号); 丢靶期间保留, 供预判参考
_coast_active = False  # 是否正在丢靶预判中
_coast_start = 0       # 预判启动时刻(time.ticks_ms())

# 上电搜索状态 —— 由 gimbal_send 维护(见 search_step)
_acquired = False      # 启动后是否已捕获过目标; 一旦置位, 上电搜索永久退出
_search_spin = False   # 是否正在搜索旋转
_search_start = 0      # 搜索启动时刻(time.ticks_ms())
_search_done = False   # 搜索超时后不再重启

# V15: 姿态关闭时退回的旧映射需要这两个方向系数; 预置默认值,
#      使电机未启用(MOTOR_ENABLED=false)时这些函数即便被引用也不会 NameError。
M_REV = 1              # yaw 指令方向系数(-1 = 反向), 初始化时按 [motor].reverse 覆盖
P_REV = 1              # pitch 同上


# ---- V15: yaw 三种状态各两套实现, 启动时二选一绑定, 热路径不留分支 ----
#   旧映射 = V14 行为(|dx|→RPM / PID / coast);  姿态 = 视觉只改 θ_ref, 转速由内环给。

def _trk_yaw_legacy(dx):
    """旧映射 yaw(attitude 关闭时): |dx| 线性映射 或 PID; 死区内停转并清状态"""
    global _last_dx, _pid_yaw_i, _pid_yaw_e, _pid_yaw_t
    _last_dx = dx
    if abs(dx) > M_DEADBAND:
        if M_PID_EN:
            _nowp = time.ticks_ms()
            _u, _pid_yaw_i, _pid_yaw_e = pid_step(
                dx, _nowp, _pid_yaw_i, _pid_yaw_e, _pid_yaw_t,
                M_PID_KP, M_PID_KI, M_PID_KD, M_MAX_RPM)
            _pid_yaw_t = _nowp
            _dirp, rpm = pid_speed(_u, M_MIN_RPM)
            if _dirp:
                _yaw.set_speed(M_REV * _dirp, rpm)
            else:
                _yaw.stop()
        else:
            ratio = min(1.0, abs(dx) / float(M_DX_FULL))
            rpm = M_MIN_RPM + (M_MAX_RPM - M_MIN_RPM) * ratio
            _yaw.set_speed(M_REV * (1 if dx > 0 else -1), rpm)
    else:
        _yaw.stop()
        _pid_clear(0)       # 死区: PID 状态清零(防再出发时积分突跳)
        _kf_clear(0)


def _trk_yaw_att(dx):
    """姿态模式 yaw: 只把像素偏差累积进方位设定值 θ_ref —— 转速交给内环"""
    global _att_ref, _att_mode, _att_d_prev, _att_d_t, _att_sdone
    now = time.ticks_ms()
    dt = _tdiff(now, _att_d_t)
    _att_d_t = now
    delta, _dn = att_outer(dx, _att_ppd, ATT_OUTER_GAIN, M_DEADBAND,
                           _att_d_prev, (dt / 1000.0) if dt > 0 else 0.0,
                           ATT_OUTER_LEAD)
    _att_d_prev = _dn
    if _att_mode != "lock":
        _att_mode = "lock"
        _att_sdone = False          # 重新锁定 = 搜索圈数计数作废(下次丢靶可再搜)
        print("ATT: lock (首次锁定, 靶心方位 %+.1f°)" % _att_ref)
    if delta != 0.0:
        _att_ref += delta


def _lost_yaw_legacy():
    """旧映射 yaw 丢靶: coast 预判(启用时)或停转"""
    global _last_dx, _coast_active, _coast_start
    _pid_clear(0)
    _kf_clear(0)
    if M_COAST_EN:
        mode, _coast_active, _coast_start, _last_dx = coast_step(
            time.ticks_ms(), _coast_active, _coast_start, _last_dx,
            M_COAST_EDGE, M_COAST_MS)
        if mode == "stop":
            print("COAST stop (timeout)")
            _yaw.stop()
        elif mode == "idle":
            _yaw.stop()
        else:
            # start / run: 朝最后方向续转, 转速=最后偏差的映射值
            ratio = min(1.0, abs(_last_dx) / float(M_DX_FULL))
            rpm = M_MIN_RPM + (M_MAX_RPM - M_MIN_RPM) * ratio
            _yaw.set_speed(M_REV * (1 if _last_dx > 0 else -1), rpm)
            if mode == "start":
                print("COAST start (last dx=%+d)" % _last_dx)
    else:
        _yaw.stop()


def _lost_yaw_att():
    """姿态模式丢靶: 保持世界方位凝视(内环把相机钉在 θ_ref 上), 超时转按圈搜索"""
    global _att_mode, _att_hold_t
    _pid_clear(0)
    _kf_clear(0)
    if _att_mode == "hold":
        return
    _att_mode = "hold"
    _att_hold_t = time.ticks_ms()
    print("ATT: hold (保持世界方位 %+.1f°, 最多 %dms)" % (_att_ref, ATT_HOLD_MS))


def _search_yaw_legacy():
    """旧映射 yaw 搜索: 单方向匀速旋转(写死 RPM; max_ms 超时后不再重启)"""
    global _search_spin, _search_start, _search_done
    _pid_clear(0)
    _kf_clear(0)
    if _search_done:
        _yaw.stop()
        return
    _prev_spin = _search_spin
    _smode, _search_spin, _search_start = search_step(
        time.ticks_ms(), _search_spin, _search_start,
        SEARCH_EN, False, SEARCH_MAX_MS)
    if _smode == "spin":
        if not _prev_spin:
            print("SEARCH start")
        _yaw.set_speed(SEARCH_DIR, SEARCH_RPM)
    elif _smode == "timeout":
        _search_done = True
        print("SEARCH stop (timeout)")
        _yaw.stop()
    else:                               # idle: 未启用
        _yaw.stop()


def _search_yaw_att():
    """姿态模式搜索: 单方向按圈扫(IMU 计 360°), 圈数用完停转且不再重启"""
    global _att_mode, _att_swept
    _pid_clear(0)
    _kf_clear(0)
    if _att_sdone:
        _yaw.stop()
        return
    if _att_mode == "search":
        return
    if not SEARCH_EN:
        _att_mode = "idle"
        _yaw.stop()
        return
    _att_mode = "search"
    _att_swept = 0.0
    print("SEARCH start (dir=%+d %.0f°/s turns=%d 从 %+.1f° 起扫)"
          % (SEARCH_DIR, SEARCH_DPS, SEARCH_TURNS, _att_ref))


def coast_step(now_ms, active, start_ms, last_dx, edge_px, coast_ms):
    """丢靶预判状态机(纯函数 —— tools/test_coast_logic.py 抓取本函数做主机侧测试)。

    返回 (mode, new_active, new_start, new_last_dx):
      "start" = 本帧启动预判(开始朝最后方向续转)   "run" = 预判进行中
      "stop"  = 预判超时终止(停转)                 "idle"= 未达触发条件
    规则: 未激活且 |last_dx| >= edge_px → 启动(记 now_ms);
          已激活且未到 coast_ms → 续跑;
          已激活且到点 → 终止, 并把 last_dx 清 0(防止超时后反复重启)。
    """
    if not active:
        if abs(last_dx) >= edge_px:
            return ("start", True, now_ms, last_dx)
        return ("idle", False, 0, last_dx)
    if _tdiff(now_ms, start_ms) < coast_ms:
        return ("run", True, start_ms, last_dx)
    return ("stop", False, 0, 0)


# ============================================================================
# 四点五、V7 视觉纯函数(主机可测) —— 画圈几何/上电搜索/引脚去抖
#   tools/test_circle_logic.py 抓取本节函数做主机侧测试(与 coast_step 同模式)
# ============================================================================

def circle_target_point(cx, cy, r, theta_deg):
    """画圈目标点(纯函数): 环心 + 半径在 θ 处的像素坐标。

    图像坐标 x 右 y 下; θ=0 指向 +x, θ 增大为视觉顺时针。
    """
    rad = math.radians(theta_deg)
    return int(round(cx + r * math.cos(rad))), int(round(cy + r * math.sin(rad)))

def circle_theta_step(theta_deg, deg_s, dt_ms, direction, err_px, pause_px,
                      wait_ms=0, wait_max_ms=0):
    """画圈 θ 推进(纯函数): 误差门控 + 到位等待超时 + 方向 + 360° 环绕。

    误差 > pause_px 时冻结不推进(等伺服追上), 同时把 dt 累计进 wait_ms;
    wait_max_ms>0 且连续未到位累计达到该值 -> 不再等待, 继续推进
    (脱机不接电机/几何偏心远点够不着时, 防 θ 永久冻结、黄点钉死在圆上不动);
    误差回到门控内 wait_ms 清零。wait_max_ms=0 = 旧行为(只门控, 永不超时)。
    dt<=0 时原样返回。返回 (theta_deg∈[0,360), wait_ms)。
    """
    if dt_ms <= 0:
        return theta_deg, wait_ms
    if err_px > pause_px:
        wait_ms += dt_ms
        if wait_max_ms <= 0 or wait_ms < wait_max_ms:
            return theta_deg, wait_ms
    else:
        wait_ms = 0
    return (theta_deg + direction * deg_s * dt_ms / 1000.0) % 360.0, wait_ms

def geo_radius_px(box):
    """V10 画圈半径(纯几何, 纯函数): 检测框 ↔ 白框物理尺寸 求像素比例, 推算同心圆半径。

    几何: 白色可见矩形(胶带内沿) RECT_W×RECT_H cm, 中心圆直径 CIRCLE_DIA cm 同心。
    用框的上边/左边像素长分别对应宽/高求 px/cm 比例取平均(自动适配框旋转90°);
    返回半径(px) 或 0(框/配置无效)。
    """
    w = math.sqrt((box[1][0] - box[0][0]) ** 2 + (box[1][1] - box[0][1]) ** 2)  # 上边 TL→TR
    h = math.sqrt((box[3][0] - box[0][0]) ** 2 + (box[3][1] - box[0][1]) ** 2)  # 左边 TL→BL
    if w <= 0 or h <= 0 or RECT_W_CM <= 0 or RECT_H_CM <= 0:
        return 0
    if (w >= h) == (RECT_W_CM >= RECT_H_CM):    # 方向一致: 宽对宽 / 高对高
        s = 0.5 * (w / RECT_W_CM + h / RECT_H_CM)
    else:                                       # 框转了90°: 宽对高 / 高对宽
        s = 0.5 * (w / RECT_H_CM + h / RECT_W_CM)
    return int(s * CIRCLE_DIA_CM * 0.5)

def circle_polyline(cx, cy, r, n=72):
    """正圆折线(纯函数; n 点供 polylines isClosed=True) —— 透视退化时的降级绘制用"""
    pts = []
    for i in range(n):
        a = 2.0 * math.pi * i / n
        pts.append((int(round(cx + r * math.cos(a))), int(round(cy + r * math.sin(a)))))
    return np.array(pts, dtype=np.int32).reshape(-1, 1, 2)

def circle_persp(quad, n=72):
    """透视圆(纯函数): 靶纸物理平面 -> 图像平面 单应投影, 圆与矩形透视一致。

    quad = order_points 环形序 [TL,TR,BR,BL](图像px), 对应物理矩形四角
    (±RECT_W_CM/2, ±RECT_H_CM/2; 物理系 y 向下, 与图像一致, θ 语义不变)。
    物理圆(半径 = CIRCLE_DIA_CM/2 + RADIUS_BIAS_CM, 与矩形同心)上 n 点经单应
    投影 -> 透视椭圆折线(橙); 附单点投影闭包供黄点定位。
    返回 (折线Nx1x2 int32, persp_fn, r_cm) 或 None(配置非法/矩阵退化/投影发散)。
    """
    if RECT_W_CM <= 0 or RECT_H_CM <= 0 or CIRCLE_DIA_CM <= 0:
        return None
    r_cm = CIRCLE_DIA_CM * 0.5 + RADIUS_BIAS_CM
    if r_cm <= 0:
        return None
    w2, h2 = RECT_W_CM * 0.5, RECT_H_CM * 0.5
    src = np.array([[-w2, -h2], [w2, -h2], [w2, h2], [-w2, h2]], dtype=np.float32)
    dst = np.array([quad[0], quad[1], quad[2], quad[3]], dtype=np.float32)
    try:
        M = cv2.getPerspectiveTransform(src, dst)
        ang = np.arange(n, dtype=np.float32) * (2.0 * math.pi / n)
        circ = np.stack([r_cm * np.cos(ang), r_cm * np.sin(ang)],
                        axis=1).astype(np.float32).reshape(-1, 1, 2)
        pts_f = cv2.perspectiveTransform(circ, M)
    except Exception:
        return None
    if not np.isfinite(pts_f).all() \
            or pts_f.min() < -100000.0 or pts_f.max() > 1000000.0:
        return None
    pts = pts_f.astype(np.int32)
    def persp(x_cm, y_cm):
        p = cv2.perspectiveTransform(
            np.array([[[float(x_cm), float(y_cm)]]], dtype=np.float32), M)
        return int(round(float(p[0, 0, 0]))), int(round(float(p[0, 0, 1])))
    return pts, persp, r_cm

def quad_area(quad):
    """四边形面积(纯函数, shoelace; quad=order_points 排序的 4 个整数顶点)。

    V8.5 几何主轨用 √面积 作框特征尺寸(距离/透视变化的等比缩放基准)。
    """
    s = 0
    x0, y0 = quad[3]
    for x, y in quad:
        s += x0 * y - x * y0
        x0, y0 = x, y
    return abs(s) // 2

def search_step(now_ms, searching, start_ms, enabled, acquired, max_ms):
    """上电搜索状态机(纯函数)。返回 (mode, searching, start_ms):
      "idle"    = 未启用 或 已捕获(不再进入搜索)
      "spin"    = 旋转中: 首次启动(记录 now_ms) / 续转(保持原 start_ms)
      "timeout" = max_ms>0 且已到点 -> 停转(不重启由调用方置 done 保证)
    max_ms=0 表示一直找, 不超时。
    """
    if not enabled or acquired:
        return "idle", False, 0
    if not searching:
        return "spin", True, now_ms
    if max_ms > 0 and _tdiff(now_ms, start_ms) >= max_ms:
        return "timeout", False, 0
    return "spin", True, start_ms

def mode_pin_step(cur_active, cur_streak, raw_active, need):
    """模式引脚去抖(纯函数): 连续 need 次采样一致才切换; 中途弹回则计数清零。

    返回 (active, streak)。raw_active 的有效电平换算在调用方完成。
    """
    if raw_active == cur_active:
        return cur_active, 0
    cur_streak += 1
    if cur_streak >= need:
        return raw_active, 0
    return cur_active, cur_streak


# ---- V16 舵机执行器: 同步下发 / 外环 / 启动 ----

def _servo_send_targets():
    """把两轴当前目标角下发: 默认 cmd25 一帧同步; 失败/关闭时逐轴 cmd13。

    内部目标是「逻辑角」(软件零点=0°), 这里统一加偏移换成舵机原始角再发。
    """
    if _sbus is None:
        return
    items = []
    if _s_ok_y and _syaw is not None:
        items.append((_syaw.id, _s_yaw_target + _s_off_y))
    if _s_ok_p and _spitch is not None:
        items.append((_spitch.id, _s_pitch_target + _s_off_p))
    if not items:
        return
    try:
        if S_SYNC and len(items) > 1:
            _sbus.send(fs_sync_mturn(items, S_INTERVAL, S_POWER))
        else:
            if _s_ok_y and _syaw is not None:
                _syaw.set_mturn(_s_yaw_target + _s_off_y, S_INTERVAL, S_POWER)
            if _s_ok_p and _spitch is not None:
                _spitch.set_mturn(_s_pitch_target + _s_off_p, S_INTERVAL, S_POWER)
    except Exception:
        pass


def _servo_boot_send(items, interval_ms):
    """启动/自检用: 按 S_SYNC 选择 cmd25 同步或逐轴 cmd13(不依赖运行期 S_INTERVAL)。

    items 是「逻辑角」列表, 函数内加软件零点偏移后下发。
    """
    if _sbus is None or not items:
        return
    raw = []
    for sid, ang in items:
        if sid == S_YAW_ID:
            raw.append((sid, ang + _s_off_y))
        elif sid == S_PITCH_ID:
            raw.append((sid, ang + _s_off_p))
        else:
            raw.append((sid, ang))
    try:
        if S_SYNC and len(raw) > 1:
            _sbus.send(fs_sync_mturn(raw, interval_ms, S_POWER))
        else:
            for sid, ang in raw:
                _sbus.send(fs_set_mturn(sid, ang, interval_ms, S_POWER))
    except Exception:
        pass


def servo_send(dx, dy, dist, status):
    """舵机版 gimbal_send: 视觉偏差 → 增量 PID → 绝对目标角 → 同步帧。

    dx/dy  : 目标中心 - 屏幕中心(px, 右/下为正)
    dist   : 像素欧氏距离(本实现不依赖; 距离增益保留为可选)
    status : "track" / "lost" / "search"
    """
    global _s_last_ms, _s_mode, _s_hold_t, _s_search_t, _s_search_dir
    global _s_search_done
    global _s_yaw_target, _s_pitch_target
    global _s_yaw_i, _s_pitch_i, _s_yaw_e, _s_pitch_e
    global _s_kf_yaw, _s_kf_pit, _s_first

    if _sbus is None or not (_s_ok_y or _s_ok_p):
        return
    try:
        now = time.ticks_ms()
        dt = 0.033 if _s_last_ms == 0 else _tdiff(now, _s_last_ms) / 1000.0
        _s_last_ms = now
        if dt < 0.005:
            dt = 0.005
        elif dt > 0.1:
            dt = 0.1

        # ---- 搜索态: yaw 有界往返扫, pitch 保持 ----
        if status == "search":
            if _s_search_done:
                _servo_send_targets()
                return
            if _s_mode != "search":
                if not S_SEARCH_EN:
                    _s_mode = "idle"
                    _s_search_done = True
                    _servo_send_targets()
                    return
                _s_mode = "search"
                _s_search_t = now
                _s_search_dir = 1 if S_SEARCH_DIR >= 0 else -1
                _s_yaw_i = 0.0
                _s_pitch_i = 0.0
                _s_kf_yaw = [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0]
                _s_kf_pit = [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0]
                _s_first = True
                print("SERVO search start (dir=%+d %.0f°/s)"
                      % (_s_search_dir, S_SEARCH_DPS))
            lo = S_YAW_MIN
            hi = S_YAW_MAX
            if S_SEARCH_RANGE > 0:
                lo = max(lo, -S_SEARCH_RANGE)
                hi = min(hi, S_SEARCH_RANGE)
            _s_yaw_target, _s_search_dir = servo_search_step(
                _s_yaw_target, _s_search_dir, S_SEARCH_DPS, dt, lo, hi)
            if S_SEARCH_MAX_MS > 0 and _tdiff(now, _s_search_t) > S_SEARCH_MAX_MS:
                _s_mode = "idle"
                _s_search_done = True
                print("SERVO search stop (timeout)")
            _servo_send_targets()
            return

        # ---- 丢靶: 先保持, 超时转有界搜索 ----
        if status == "lost":
            if _s_mode != "hold" and _s_mode != "search":
                _s_mode = "hold"
                _s_hold_t = now
                _s_yaw_i = 0.0
                _s_pitch_i = 0.0
                _s_kf_yaw = [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0]
                _s_kf_pit = [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0]
                _s_first = True
                print("SERVO hold (保持最后目标角, 最多 %dms)" % S_HOLD_MS)
            if _s_mode == "hold" and (S_HOLD_MS <= 0
                                      or _tdiff(now, _s_hold_t) > S_HOLD_MS) \
                    and S_SEARCH_EN and not _s_search_done:
                _s_mode = "search"
                _s_search_t = now
                _s_search_dir = 1 if S_SEARCH_DIR >= 0 else -1
                print("SERVO hold timeout -> search")
            if _s_mode == "search":
                if S_SEARCH_MAX_MS > 0 and _tdiff(now, _s_search_t) > S_SEARCH_MAX_MS:
                    _s_mode = "hold"
                    _s_search_done = True
                    print("SERVO search stop (timeout)")
                else:
                    lo = S_YAW_MIN
                    hi = S_YAW_MAX
                    if S_SEARCH_RANGE > 0:
                        lo = max(lo, -S_SEARCH_RANGE)
                        hi = min(hi, S_SEARCH_RANGE)
                    _s_yaw_target, _s_search_dir = servo_search_step(
                        _s_yaw_target, _s_search_dir, S_SEARCH_DPS, dt, lo, hi)
            _servo_send_targets()
            return

        # ---- 追踪: 两轴增量 PID ----
        if _s_mode != "track":
            _s_yaw_i = 0.0
            _s_pitch_i = 0.0
            _s_kf_yaw = [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0]
            _s_kf_pit = [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0]
            _s_first = True
            _s_search_done = False
        _s_mode = "track"
        if S_KF_EN:
            ex = kf_step(dx, now, _s_kf_yaw, S_KF_QPOS, S_KF_QVEL,
                         S_KF_R, S_KF_LEAD, S_KF_LIM)
            vx = _s_kf_yaw[1]
            ey = kf_step(dy, now, _s_kf_pit, S_KF_QPOS, S_KF_QVEL,
                         S_KF_R, S_KF_LEAD, S_KF_LIM)
            vy = _s_kf_pit[1]
        else:
            ex = float(dx)
            ey = float(dy)
            vx = None
            vy = None

        if _s_first:
            # 首帧/重捕获: 用当前误差初始化 e_prev, 避免 D=(e-0)/dt 的初始 kick
            _s_yaw_e = ex
            _s_pitch_e = ey
            _s_first = False

        # yaw
        _iy0 = _s_yaw_i
        _ux, _s_yaw_i, _s_yaw_e = servo_pid(
            ex, _s_yaw_e, _s_yaw_i, dt,
            S_YAW_KP, S_YAW_KI if S_PID_EN else 0.0,
            S_YAW_KD if S_PID_EN else 0.0,
            S_YAW_KFF if S_PID_EN else 0.0,
            vx, S_MAX_DPS, S_DEADBAND, S_IZONE)
        _ux = servo_boost(_ux, ex, S_BOOST_PX, S_MAX_DPS)
        _nt, _sat = servo_step_target(
            _s_yaw_target, S_YAW_SIGN * _ux, dt,
            S_YAW_MIN, S_YAW_MAX, S_MAX_STEP)
        if _sat:
            _s_yaw_i = _iy0          # 饱和冻结积分(抗 windup)
        _s_yaw_target = _nt

        # pitch
        _ip0 = _s_pitch_i
        _uy, _s_pitch_i, _s_pitch_e = servo_pid(
            ey, _s_pitch_e, _s_pitch_i, dt,
            S_PITCH_KP, S_PITCH_KI if S_PID_EN else 0.0,
            S_PITCH_KD if S_PID_EN else 0.0,
            S_PITCH_KFF if S_PID_EN else 0.0,
            vy, S_MAX_DPS, S_DEADBAND, S_IZONE)
        _uy = servo_boost(_uy, ey, S_BOOST_PX, S_MAX_DPS)
        _nt, _sat = servo_step_target(
            _s_pitch_target, S_PITCH_SIGN * _uy, dt,
            S_PITCH_MIN, S_PITCH_MAX, S_MAX_STEP)
        if _sat:
            _s_pitch_i = _ip0
        _s_pitch_target = _nt

        _servo_send_targets()
    except Exception:
        pass


def _servo_init():
    """打开单总线, ping/读当前角, 设定初始目标(不跳变), 打印自检行。"""
    global _sbus, _syaw, _spitch, _s_ok_y, _s_ok_p
    global _s_yaw_target, _s_pitch_target, _gimbal_ok
    global _s_off_y, _s_off_p

    print("act: servo bus UART%d open (%s/%s) ..." % (S_PORT, S_TX, S_RX))
    _sbus = FashionStarBus(S_PORT, S_BAUD, S_TX, S_RX, 80)
    _syaw = FashionStarServo(_sbus, S_YAW_ID)
    _spitch = FashionStarServo(_sbus, S_PITCH_ID)

    _s_ok_y = _syaw.ping()
    print("act: servo yaw  id=%d %s" % (S_YAW_ID, "online" if _s_ok_y else "NO RESP"))
    time.sleep_ms(10)
    _s_ok_p = _spitch.ping()
    print("act: servo pitch id=%d %s" % (S_PITCH_ID, "online" if _s_ok_p else "NO RESP"))

    ay = _syaw.read_mturn() if _s_ok_y else None
    if _s_ok_y and ay is None:
        print("act: WARN yaw ping ok but angle read failed -> yaw 不动作(防跳变)")
        _s_ok_y = False
    time.sleep_ms(10)
    ap = _spitch.read_mturn() if _s_ok_p else None
    if _s_ok_p and ap is None:
        print("act: WARN pitch ping ok but angle read failed -> pitch 不动作(防跳变)")
        _s_ok_p = False

    if S_ZERO_AUTO:
        # 软件零点: 把「现在这个姿态」记成逻辑 0°(偏移=原始多圈角), 不写舵机 EEPROM
        _s_off_y = ay if ay is not None else 0.0
        _s_off_p = ap if ap is not None else 0.0
        print("act: servo 软件零点(auto) yaw raw=%s pitch raw=%s -> 逻辑 0°"
              % ("-" if ay is None else "%.1f" % ay,
                 "-" if ap is None else "%.1f" % ap))
    else:
        _s_off_y = float(S_YAW_ZERO)
        _s_off_p = float(S_PITCH_ZERO)
        if _s_off_y or _s_off_p:
            print("act: servo 软件零点(fixed) yaw=%+.1f° pitch=%+.1f°"
                  % (_s_off_y, _s_off_p))

    _s_yaw_target = (ay - _s_off_y) if ay is not None else 0.0
    _s_pitch_target = (ap - _s_off_p) if ap is not None else 0.0

    if S_YAW_MIN >= S_YAW_MAX:
        print("act: WARN yaw_min >= yaw_max (%.1f/%.1f) -> yaw 不动作"
              % (S_YAW_MIN, S_YAW_MAX))
        _s_ok_y = False
    if S_PITCH_MIN >= S_PITCH_MAX:
        print("act: WARN pitch_min >= pitch_max (%.1f/%.1f) -> pitch 不动作"
              % (S_PITCH_MIN, S_PITCH_MAX))
        _s_ok_p = False

    # 软限位保护: 当前角已在限位外时不要下发(否则第一帧会 clamp 出一个大跳变)。
    # 坐标是「逻辑角」(软件零点之后); 常见原因: 软件零点没设 / yaw_min/max 比机械行程小。
    if _s_ok_y and (_s_yaw_target < S_YAW_MIN - 0.5 or _s_yaw_target > S_YAW_MAX + 0.5):
        print("act: WARN yaw 当前逻辑角 %.1f° (raw %s) 在软限位 [%.0f, %.0f] 外 ->"
              " yaw 不动作; 请设 [servo].zero_auto/yaw_zero_deg 或改 yaw_min/max" %
              (_s_yaw_target, "-" if ay is None else "%.1f" % ay,
               S_YAW_MIN, S_YAW_MAX))
        _s_ok_y = False
    if _s_ok_p and (_s_pitch_target < S_PITCH_MIN - 0.5 or _s_pitch_target > S_PITCH_MAX + 0.5):
        print("act: WARN pitch 当前逻辑角 %.1f° (raw %s) 在软限位 [%.0f, %.0f] 外 ->"
              " pitch 不动作; 请设 [servo].zero_auto/pitch_zero_deg 或改 pitch_min/max" %
              (_s_pitch_target, "-" if ap is None else "%.1f" % ap,
               S_PITCH_MIN, S_PITCH_MAX))
        _s_ok_p = False

    if S_BOOT_CENTER:
        items = []
        if _s_ok_y:
            _s_yaw_target = 0.0
            items.append((_syaw.id, 0.0))
        if _s_ok_p:
            _s_pitch_target = 0.0
            items.append((_spitch.id, 0.0))
        if items:
            _servo_boot_send(items, S_CENTER_MS)
            print("act: servo boot center -> 0° (%dms 慢速)" % S_CENTER_MS)
    else:
        # 以当前角为初始目标并锁力, 避免上电跳变
        items = []
        if _s_ok_y:
            items.append((_syaw.id, _s_yaw_target))
        if _s_ok_p:
            items.append((_spitch.id, _s_pitch_target))
        if items:
            _servo_boot_send(items, 0)

    print("actuator: yaw=servo(id=%d %s angle=%.1f [%+.0f,%+.0f] zero=%+.1f) | "
          "pitch=servo(id=%d %s angle=%.1f [%+.0f,%+.0f] zero=%+.1f)"
          % (S_YAW_ID, "online" if _s_ok_y else "offline", _s_yaw_target,
             S_YAW_MIN, S_YAW_MAX, _s_off_y,
             S_PITCH_ID, "online" if _s_ok_p else "offline", _s_pitch_target,
             S_PITCH_MIN, S_PITCH_MAX, _s_off_p))
    _gimbal_ok = _s_ok_y or _s_ok_p
    if not _gimbal_ok:
        print("!!! servo: 两轴都不在线 -> gimbal_send 空操作(查供电/共地/1k/ID/波特率)")


def _servo_self_test():
    """servo.test_mode: 不跑视觉, 只做 ping/读角/±5° 点动/同步帧自检。"""
    print(">>> SERVO TEST (FashionStar 总线舵机, 不跑视觉; Ctrl+C/MaixVision 退出)")
    print("  bus: UART%d @%dbps TX=%s RX=%s | yaw=ID%d pitch=ID%d"
          % (S_PORT, S_BAUD, S_TX, S_RX, S_YAW_ID, S_PITCH_ID))
    time.sleep_ms(200)
    try:
        for _nm, _sv, _ok, _lo, _hi, _off in (
                ("yaw", _syaw, _s_ok_y, S_YAW_MIN, S_YAW_MAX, _s_off_y),
                ("pitch", _spitch, _s_ok_p, S_PITCH_MIN, S_PITCH_MAX, _s_off_p)):
            if not _ok or _sv is None:
                print("  %s: offline, 跳过点动" % _nm)
                continue
            _r0 = _sv.read_mturn()
            if _r0 is None:
                print("  %s: 读角失败, 跳过点动" % _nm)
                continue
            _a0 = _r0 - _off                      # 逻辑角
            _a1 = min(_hi, max(_lo, _a0 + 5.0))
            print("  %s: 逻辑 %.1f° (raw %.1f) -> %.1f° -> %.1f°"
                  % (_nm, _a0, _r0, _a1, _a0))
            _sv.set_mturn(_a1 + _off, 600, S_POWER)
            time.sleep_ms(800)
            _sv.set_mturn(_a0 + _off, 600, S_POWER)
            time.sleep_ms(800)
            _r1 = _sv.read_mturn()
            print("  %s: 回读 逻辑 %s° (raw %s)"
                  % (_nm,
                     "-" if _r1 is None else "%.1f" % (_r1 - _off),
                     "-" if _r1 is None else "%.1f" % _r1))
        # 同步帧: 两轴各 +5° 再回
        if _s_ok_y and _s_ok_p:
            _ay = min(S_YAW_MAX, max(S_YAW_MIN, _s_yaw_target + 5.0))
            _ap = min(S_PITCH_MAX, max(S_PITCH_MIN, _s_pitch_target + 5.0))
            print("  sync: yaw %.1f° pitch %.1f° (cmd25)" % (_ay, _ap))
            _servo_boot_send([(_syaw.id, _ay), (_spitch.id, _ap)], S_CENTER_MS)
            time.sleep_ms(S_CENTER_MS + 300)
            _servo_boot_send([(_syaw.id, _s_yaw_target),
                              (_spitch.id, _s_pitch_target)], S_CENTER_MS)
            time.sleep_ms(S_CENTER_MS + 300)
            print("  sync: 回读 yaw=%.1f° pitch=%.1f°"
                  % (_syaw.read_mturn(), _spitch.read_mturn()))
    except Exception as _e:
        print("  SERVO TEST error: %s" % _e)
    print(">>> SERVO TEST done (保持锁力; 退出请 Ctrl+C)")
    while not app.need_exit():
        time.sleep_ms(500)


try:
    print("boot: 执行器初始化 ...")
    # ---------------- pitch 上下轴: Emm42 闭环步进(UART3, 速度模式, 与 yaw 同逻辑) ----------------
    if PITCH_ENABLED and not SERVO_ENABLED:
        P_PORT = pm.get("port", 3)          # UART 控制器编号(/dev/ttyS3)
        P_BAUD = pm.get("baud", 115200)     # 波特率
        P_TX = pm.get("tx_pin", "B2")       # MaixCAM2 TX(引脚名) -> 驱动器② RXD
        P_RX = pm.get("rx_pin", "B3")       # MaixCAM2 RX(引脚名) -> 驱动器② TXD
        P_ADDR = pm.get("addr", 1)          # 驱动器串口 ID
        # P_ACC/P_MIN_RPM/P_MAX_RPM/P_DY_FULL/P_DEADBAND = 模块级"活参数"(V7.9 触屏可调, 见第二节)
        P_REV = -1 if pm.get("reverse", False) else 1  # 方向翻转

        # 一个 UART 控制器只能有一组引脚, 两轴配成同一个 port 会互相覆盖
        if MOTOR_ENABLED and mv.get("port", 2) == P_PORT:
            print("  WARN: yaw 与 pitch 都配在 UART%d 上, 会互相覆盖!" % P_PORT)
            print("        → 把其中一条改到别的 UART, 或把对应轴 enabled 置 false")

        print("act: pitch UART%d open (%s/%s) ..." % (P_PORT, P_TX, P_RX))
        _pitch = EmmStepper(P_PORT, P_BAUD, P_TX, P_RX, P_ADDR, P_ACC)
        print("act: pitch uart ok")
        print("act: pitch enabling (首次向 UART%d 写指令) ..." % P_PORT)
        if pm.get("enable_on_boot", True):
            _pitch.enable(True)     # 使能电机(不使能的话收到速度指令也不转)
        print("act: pitch enable sent, stopping ...")
        _pitch.stop()               # 上电先停住, 防止误转
        print("act: pitch ready")

    # ---------------- yaw 左右轴: Emm42 闭环步进(UART2) ----------------
    if MOTOR_ENABLED and not SERVO_ENABLED:
        M_PORT = mv.get("port", 2)          # UART 控制器编号(/dev/ttyS2)
        M_BAUD = mv.get("baud", 115200)     # 波特率
        M_TX = mv.get("tx_pin", "B0")       # MaixCAM2 TX(引脚名) -> 驱动器 RXD
        M_RX = mv.get("rx_pin", "B1")       # MaixCAM2 RX(引脚名) -> 驱动器 TXD
        M_ADDR = mv.get("addr", 1)          # 驱动器串口 ID
        # M_ACC/M_MIN_RPM/M_MAX_RPM/M_DX_FULL/M_DEADBAND = 模块级"活参数"(V7.9 触屏可调, 见第二节)
        M_REV = -1 if mv.get("reverse", False) else 1
        M_COAST_EN = mv.get("coast_enabled", False)    # 丢靶预判开关(默认关, 先自检验证再启用)
        M_COAST_EDGE = mv.get("coast_edge_px", 160)    # 触发门槛: 最后 |dx| >= 此值
        M_COAST_MS = mv.get("coast_ms", 800)           # 预判持续时长(ms)

        print("act: yaw UART%d open (%s/%s) ..." % (M_PORT, M_TX, M_RX))
        _yaw = EmmStepper(M_PORT, M_BAUD, M_TX, M_RX, M_ADDR, M_ACC)
        print("act: yaw uart ok")
        print("act: yaw enabling (首次向 UART%d 写指令) ..." % M_PORT)
        if mv.get("enable_on_boot", True):
            _yaw.enable(True)       # 使能电机(不使能的话收到速度指令也不转)
        print("act: yaw enable sent, stopping ...")
        _yaw.stop()                 # 上电先停住, 防止误转
        print("act: yaw ready")

    # ---------------- V15 姿态传感器: 板载 IMU(LSM6DSOW, 免接线, 与相机同一刚体) ----------------
    #   即使 attitude.enabled=false 也初始化一次: θ/ω 直接显示在 OSD 上(只读取证),
    #   触屏也能随时把 att 开关打开, 不必重启。任何失败都不影响旧映射追踪。
    if _imu_mod is not None:
        try:
            for _i in _imu_mod.get_imu_info():
                print("att: IMU %s driver=%s i2c%d addr=0x%02X mag=%s"
                      % (_i.name, _i.driver, _i.i2c_bus, _i.addr, _i.have_mag))
            _gs_cls = _imu_mod.GyroScale
            _gscale = getattr(_gs_cls, "GYRO_SCALE_%dDPS" % ATT_GYRO_DPS, None)
            if _gscale is None:
                _gscale = _gs_cls.GYRO_SCALE_2000DPS
            _imu = _imu_mod.IMU("", mode=_imu_mod.Mode.DUAL,
                                acc_scale=_imu_mod.AccScale.ACC_SCALE_2G,
                                gyro_scale=_gscale,
                                gyro_odr=_imu_mod.GyroOdr.GYRO_ODR_2000)
            if _imu.calib_gyro_exists():
                _imu.load_calib_gyro()
                print("att: IMU ready (gyro ±%d dps, 零偏已加载)" % ATT_GYRO_DPS)
            else:
                print("att: 首次使用 -> 陀螺零偏校准, 请保持云台静止 10 秒 ...")
                _imu.calib_gyro(10000)
                print("att: IMU ready (gyro ±%d dps, 零偏已校准并存盘)" % ATT_GYRO_DPS)
        except Exception as _e:
            print("att: IMU 初始化失败 (%s) -> 姿态功能不可用, 旧映射照常" % _e)
            _imu = None
    else:
        print("att: 固件无 maix.ext_dev.imu -> 姿态功能不可用, 旧映射照常")

    # ---- 绑定 yaw 实现: IMU 就绪则内环常驻(θ/ω 可观测); attitude 开关为真才接管电机 ----
    _att_cmd_sign = M_REV * ATT_YAW_SIGN
    if _imu is not None:
        _att_spin = _att_spin_run
    if SERVO_ENABLED and ATT_EN_CFG:
        # 舵机是位置模式, V15 的 650Hz RPM 内环会打爆 115200 总线, 且舵机内部位置环
        # 已经承担快速跟踪; 世界系稳定后续用 30~50Hz 的目标角修正单独做。
        ATT_EN_CFG = False
        print("att: servo 模式关闭 650Hz 姿态内环(总线带宽/舵机内环) -> 走视觉外环")
    if ATT_EN_CFG and _imu is not None:
        _att_bind(True)
    else:
        ATT_EN = False
        _att_drive = False
        _trk_yaw = _trk_yaw_legacy
        _lost_yaw = _lost_yaw_legacy
        _search_yaw = _search_yaw_legacy
        print("att: 姿态未接管(%s) —— 走 [motor] 旧映射; OSD 仍显示 θ/ω"
              % ("配置关闭" if not ATT_EN_CFG else "无 IMU"))

    def gimbal_send(dx, dy, dist, status):
        """每帧调用一次, 把视觉偏差送给两条轴

        dx, dy : 目标中心相对屏幕中心的像素偏差(右偏/下偏为正)
        dist   : 偏差欧氏距离(本实现未用到, 仅为接口完整保留)
        status : "track"=追踪中 / "lost"=丢靶 / "search"=上电搜索(yaw 匀速找靶)

        丢靶预判(coast): 丢靶且最后偏差 |_last_dx| >= coast_edge_px 时,
        不立即停, 而是朝最后方向续转 coast_ms 毫秒(无目标追踪)去找目标;
        超时停转, 期间一旦重新捕获(track)立即恢复常规追踪。
        pitch 轴不参与预判(丢靶即停, 与旧行为一致)。
        """
        global _last_dx, _coast_active
        global _acquired
        global _pid_pit_i, _pid_pit_e, _pid_pit_t
        try:
            if status == "search":
                # 搜索(仅"启动后未捕获过目标"期间由 run_cv 调用):
                #   旧映射 = yaw 单方向匀速 RPM;  姿态 = 单方向按圈扫(IMU 计 360°)
                _pid_clear(1)               # pitch 非追踪态: PID/KF 状态清零
                _kf_clear(1)
                if _yaw:
                    _search_yaw()           # yaw 侧的状态清零在实现内部做
                if _pitch:
                    _pitch.stop()
                return
            if status == "lost":
                _pid_clear(1)               # pitch 丢靶: 状态清零
                _kf_clear(1)
                if _yaw:
                    _lost_yaw()             # 旧=coast 预判/停转;  姿态=保持世界方位→超时转搜索
                if _pitch:
                    _pitch.stop()           # 丢靶: pitch 一律停转
                return

            # ---- 追踪中: 记录最后偏差(供丢靶预判), 复位预判状态, 标记已捕获 ----
            # ---- (V8) KF 误差滤波: kf_enabled=true 时平滑误差 + 速度前馈; 默认关=原样直通 ----
            if KF_Y_EN or KF_P_EN:
                _nowk = time.ticks_ms()
                if KF_Y_EN:
                    dx = int(kf_step(dx, _nowk, _kf_yaw, KF_Y_QPOS, KF_Y_QVEL,
                                     KF_Y_R, KF_Y_LEAD, KF_Y_LIM))
                if KF_P_EN:
                    dy = int(kf_step(dy, _nowk, _kf_pit, KF_P_QPOS, KF_P_QVEL,
                                     KF_P_R, KF_P_LEAD, KF_P_LIM))
            _last_dx = dx
            _coast_active = False
            _acquired = True     # 上电搜索: 一旦追踪过目标, 永久退出搜索

            # ---- yaw: 两套实现二选一(旧「|dx|映射/PID」 或 「姿态内环」), 指针启动时已绑好 ----
            # 旧实现里的 M_PID_EN 运行时判定保留在 _trk_yaw_legacy 内(触屏可实时开关)。
            if _yaw:
                _trk_yaw(dx)

            # ---- pitch: 同 yaw(PID 或原映射, 死区内停) ----
            if _pitch:
                if abs(dy) > P_DEADBAND:
                    if P_PID_EN:
                        _nowp = time.ticks_ms()
                        _u, _pid_pit_i, _pid_pit_e = pid_step(
                            dy, _nowp, _pid_pit_i, _pid_pit_e, _pid_pit_t,
                            P_PID_KP, P_PID_KI, P_PID_KD, P_MAX_RPM)
                        _pid_pit_t = _nowp
                        _dirp, rpm = pid_speed(_u, P_MIN_RPM)
                        if _dirp:
                            _pitch.set_speed(P_REV * _dirp, rpm)
                        else:
                            _pitch.stop()
                    else:
                        ratio = min(1.0, abs(dy) / float(P_DY_FULL))
                        rpm = P_MIN_RPM + (P_MAX_RPM - P_MIN_RPM) * ratio
                        _pitch.set_speed(P_REV * (1 if dy > 0 else -1), rpm)
                else:
                    _pitch.stop()
                    _pid_clear(1)
                    _kf_clear(1)        # V8: KF 状态同步清零
        except Exception:
            pass

    # ---- 打印执行器状态(上电自检的第一手信息) ----
    # V8: 可选上电清堵转(驱动器因撞限位/卡死进过堵转保护时恢复; 默认关)
    if M_CLOG_RESET and _yaw:
        _yaw.reset_clog()
        print("clog reset sent (yaw)")
    if P_CLOG_RESET and _pitch:
        _pitch.reset_clog()
        print("clog reset sent (pitch)")
    _yaw_info = "off"
    if _yaw:
        _v = _yaw.read_bus_voltage()    # 兼作在线探测: 有回包说明驱动器和串口都通
        _rpm0 = _yaw.read_velocity() if _v else None   # V8: 顺带验证读取链路(上电应静止)
        _yaw_info = "emm(addr=%d UART%d TX=%s RX=%s %d~%dRPM rev=%s)%s%s" % (
            M_ADDR, M_PORT, M_TX, M_RX, M_MIN_RPM, M_MAX_RPM,
            "Y" if M_REV < 0 else "N",
            (" online %dmV" % _v) if _v else " (no resp)",
            (" rpm=%d" % _rpm0) if _rpm0 is not None else "")
    _pitch_info = "off"
    if _pitch:
        _v = _pitch.read_bus_voltage()      # 兼作在线探测: 有回包说明驱动器和串口都通
        _rpm0 = _pitch.read_velocity() if _v else None  # V8: 同上
        _pitch_info = "emm(addr=%d UART%d TX=%s RX=%s %d~%dRPM rev=%s)%s%s" % (
            P_ADDR, P_PORT, P_TX, P_RX, P_MIN_RPM, P_MAX_RPM,
            "Y" if P_REV < 0 else "N",
            (" online %dmV" % _v) if _v else " (no resp)",
            (" rpm=%d" % _rpm0) if _rpm0 is not None else "")
    if not SERVO_ENABLED:
        print("actuator: yaw=%s | pitch=%s" % (_yaw_info, _pitch_info))

    # ---------------- V7 模式引脚(默认 A4): 被供电=画圈, 其余=仅追踪 ----------------
    if PIN_ENABLED and PIN_GPIO:
        try:
            pinmap.set_pin_function(PIN_GPIO, "GPIO%s" % PIN_GPIO)
            _mode_pin = gpio.GPIO("GPIO%s" % PIN_GPIO, gpio.Mode.IN)
            _pv = _mode_pin.value()
            _pa = ((_pv != 0) if PIN_ACTIVE == 1 else (_pv == 0))
            print("mode pin: %s level=%d -> %s" % (PIN_GPIO, _pv,
                  "画圈" if _pa else "仅追踪"))
        except Exception as e:
            print("WARN: mode pin init failed: %s -> 永久仅追踪" % e)
            _mode_pin = None

    _gimbal_ok = True    # 执行器就绪: gimbal_send 为真实实现(供 coast_test 等自检判断)

except Exception as e:
    print("\n!!! actuator init failed: %s" % str(e))
    gimbal_send = lambda dx, dy, d, s: None


# ---- V16: 舵机路径接管(在步进 try 之后, 成功则重绑 gimbal_send) ----
if SERVO_ENABLED:
    try:
        _servo_init()
        gimbal_send = servo_send
    except Exception as e:
        print("\n!!! servo init failed: %s" % str(e))
        _gimbal_ok = False
        gimbal_send = lambda dx, dy, d, s: None


# ============================================================================
# 五、CV 矩形检测
# ============================================================================

def point_xy(point):
    try:
        return int(point[0][0]), int(point[0][1])
    except Exception:
        return int(point[0]), int(point[1])

def order_points(points):
    pts = [point_xy(p) for p in points]
    cx = sum(p[0] for p in pts) // 4
    cy = sum(p[1] for p in pts) // 4
    top = [p for p in pts if p[1] < cy]
    bottom = [p for p in pts if p[1] >= cy]
    if len(top) != 2:
        pts.sort(key=lambda p: p[1])
        top, bottom = pts[:2], pts[2:]
    top.sort(key=lambda p: p[0])
    bottom.sort(key=lambda p: p[0])
    return [top[0], top[1], bottom[1], bottom[0]]

def get_contours(result):
    return result[0] if len(result) == 2 else result[1]

def quad_center(box):
    """四边形对角线交点(精确中心)"""
    x1, y1 = box[0]; x2, y2 = box[2]; x3, y3 = box[1]; x4, y4 = box[3]
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if den == 0:
        return (sum(p[0] for p in box) // 4, sum(p[1] for p in box) // 4)
    pre = x1 * y2 - y1 * x2
    post = x3 * y4 - y3 * x4
    cx = (pre * (x3 - x4) - (x1 - x2) * post) // den
    cy = (pre * (y3 - y4) - (y1 - y2) * post) // den
    return int(cx), int(cy)

def side_len2(a, b):
    dx, dy = a[0] - b[0], a[1] - b[1]
    return dx * dx + dy * dy

def is_good_quad(quad, contour_area, box_area):
    if contour_area * 100 < box_area * MIN_RECT_FILL:
        return False
    sides = [side_len2(quad[i], quad[(i + 1) % 4]) for i in range(4)]
    if min(sides) <= 0 or max(sides) > min(sides) * MAX_SIDE_RATIO * MAX_SIDE_RATIO:
        return False
    h_avg = (sides[0] + sides[2]) // 2
    v_avg = (sides[1] + sides[3]) // 2
    if min(h_avg, v_avg) <= 0:
        return False
    ratio = max(h_avg, v_avg) / min(h_avg, v_avg)
    if ratio > MAX_ASPECT * MAX_ASPECT or ratio < MIN_ASPECT * MIN_ASPECT:
        return False
    return True

def contour_quad(cnt, box_area):
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, APPROX_EPSILON * peri, True)
    if len(approx) != 4:
        return None
    quad = order_points(approx)
    if not is_good_quad(quad, cv2.contourArea(cnt), box_area):
        return None
    return quad

def has_black_border(frame_np, x, y, w, h):
    if w < 20 or h < 20:
        return False
    black = cv2.inRange(frame_np, BLACK_LOW, BLACK_HIGH)
    edge = max(2, min(w, h) // 18)
    top = black[y:y + edge, x:x + w]
    bottom = black[y + h - edge:y + h, x:x + w]
    left = black[y:y + h, x:x + edge]
    right = black[y:y + h, x + w - edge:x + w]
    hits = 0
    if cv2.countNonZero(top) > w // BLACK_CHECK_STEP: hits += 1
    if cv2.countNonZero(bottom) > w // BLACK_CHECK_STEP: hits += 1
    if cv2.countNonZero(left) > h // BLACK_CHECK_STEP: hits += 1
    if cv2.countNonZero(right) > h // BLACK_CHECK_STEP: hits += 1
    return hits >= MIN_BLACK_HITS

def has_white_center(white_mask, x, y, w, h):
    ix, iy = x + w // 4, y + h // 4
    iw, ih = w // 2, h // 2
    if iw <= 0 or ih <= 0:
        return False
    roi = white_mask[iy:iy + ih, ix:ix + iw]
    return cv2.countNonZero(roi) > (iw * ih) // CENTER_WHITE_STEP

def clamp_box(x, y, w, h):
    if x < 0: x = 0
    if y < 0: y = 0
    if x + w > SENSOR_W: w = SENSOR_W - x
    if y + h > SENSOR_H: h = SENSOR_H - y
    return x, y, w, h

def find_paper_box(frame_np, last_box):
    """检测矩形靶标, 返回 (box4点, 轮廓数, 面积候选数)"""
    mask = cv2.inRange(frame_np, WHITE_LOW, WHITE_HIGH)
    contours = get_contours(cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))
    best, best_score = None, -1
    area_count = 0
    last_cx = last_cy = last_area = 0
    if last_box:
        last_cx, last_cy = quad_center(last_box)
        xs = [p[0] for p in last_box]; ys = [p[1] for p in last_box]
        last_area = (max(xs) - min(xs)) * (max(ys) - min(ys))
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        if w <= 0 or h <= 0:
            continue
        box_area = w * h
        if box_area < MIN_AREA or box_area > MAX_AREA:
            continue
        area_count += 1
        aspect = w / h
        if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
            continue
        x -= BORDER_EXPAND_X; y -= BORDER_EXPAND_Y
        w += BORDER_EXPAND_X * 2; h += BORDER_EXPAND_Y * 2
        x, y, w, h = clamp_box(x, y, w, h)
        cx, cy = x + w // 2, y + h // 2
        if last_box and abs(cx - last_cx) + abs(cy - last_cy) > MAX_CENTER_JUMP:
            continue
        if last_area and (box_area > last_area * MAX_AREA_RATIO or last_area > box_area * MAX_AREA_RATIO):
            continue
        if not has_white_center(mask, x, y, w, h):
            continue
        if not has_black_border(frame_np, x, y, w, h):
            continue
        quad = contour_quad(cnt, box_area)
        if not quad:
            continue
        score = box_area - (abs(cx - last_cx) + abs(cy - last_cy)) * 8 if last_box else box_area
        if score > best_score:
            best_score = score
            best = quad
    return best, len(contours), area_count

def draw_box(frame_np, box):
    """绘制矩形框+中心十字, 返回 (cx, cy, dx, dy)"""
    for i in range(4):
        cv2.line(frame_np, box[i], box[(i + 1) % 4], (0, 255, 0), 2)
    cx, cy = quad_center(box)
    dx, dy = cx - CENTER_X, cy - CENTER_Y
    cv2.line(frame_np, (cx - 14, cy), (cx + 14, cy), (255, 255, 0), 2)
    cv2.line(frame_np, (cx, cy - 14), (cx, cy + 14), (255, 255, 0), 2)
    cv2.circle(frame_np, (cx, cy), 4, (0, 0, 255), 1)
    return cx, cy, dx, dy


# ---- V9 距离估算与激光点映射(移植自参考视觉 v2.3) ----
#   面积法测距: D = D_ref × √(A_ref / A)   (同一靶标, 越近像素面积越大)
#   激光点映射: X 固定(激光器安装偏移); Y = Y_near + (D-D_min)×(Y_far-Y_near)/(D_max-D_min), D 先限幅

def estimate_distance_by_area(area_px):
    """面积法估算靶标距离(mm); area_px<=0 返回 0.0(无效)

    标定: 把靶标放在 ref_distance_mm 处, 检测面积填入 ref_area_px。
    """
    if area_px <= 0:
        return 0.0
    return DIST_REF_MM * math.sqrt(DIST_REF_AREA / area_px)

def laser_position_by_distance(distance_mm):
    """距离线性映射激光点坐标 → (x, y); 距离先限幅到 [DIST_MIN, DIST_MAX]"""
    if DIST_MAX <= DIST_MIN:
        return (LASER_X, LASER_Y_NEAR)       # 配置防呆: 区间非法时固定近点
    d = distance_mm
    if d < DIST_MIN:
        d = DIST_MIN
    elif d > DIST_MAX:
        d = DIST_MAX
    y = LASER_Y_NEAR + (d - DIST_MIN) * (LASER_Y_FAR - LASER_Y_NEAR) / (DIST_MAX - DIST_MIN)
    return (LASER_X, int(y))


# ============================================================================
# 六、YOLO 检测支持
# ============================================================================

def _yolo_ok():
    """V11: YOLO 在 MaixCAM2 上待移植(maix.nn + AX 格式模型), 恒回退 cv。"""
    return False


# ============================================================================
# 六点五、触屏中心校准
#   触摸按键移动绿色校准点(追踪收敛点), 保存写回 config
#   V11 显示为 FIT_CONTAIN 居中显示: 触摸屏幕坐标经 resize_map_pos_reverse 映射回图像系
# ============================================================================

def _calib_rects(ox, oy, sw, sh, bw):
    """生成 8 个触摸按钮的命中矩形(LCD 坐标): 上/下(左缘) 左/右(右缘)
    mode追踪/画圈切换 · dot隐藏 · S保存 · P调参页(顶中一字排开)"""
    m = 6
    y1 = sh // 2 - bw - 6
    y2 = y1 + bw + 12
    xl = ox + m
    xr = ox + sw - m - bw
    xc = ox + sw // 2
    return (
        ("up",    xl, oy + y1, xl + bw, oy + y1 + bw),
        ("down",  xl, oy + y2, xl + bw, oy + y2 + bw),
        ("left",  xr, oy + y1, xr + bw, oy + y1 + bw),
        ("right", xr, oy + y2, xr + bw, oy + y2 + bw),
        ("mode",  xc - 2 * bw - 12, oy + m, xc - bw - 12, oy + m + bw),
        ("dot",   xc - bw - 4, oy + m, xc - 4, oy + m + bw),
        ("save",  xc + 4, oy + m, xc + bw + 4, oy + m + bw),
        ("param", xc + bw + 12, oy + m, xc + 2 * bw + 12, oy + m + bw),
    )

def calib_hit(x, y, rects):
    """触摸点命中的按钮名; 未命中返回 None(边界视为命中)"""
    for name, x0, y0, x1, y1 in rects:
        if x0 <= x <= x1 and y0 <= y <= y1:
            return name
    return None

def calib_step(vis, btn, hold_t0, last_btn, now, pressed, was_pressed, hit, repeat_ms, guard_ms, act_t):
    """校准 UI 触摸状态机(每帧一次)。返回 (vis, btn, hold_t0, last_btn, act_t, action)。

    规则: 隐藏态仅"新触摸边沿"唤出(本次触摸被消费, 不发动作; 防点 dot 未松手即弹回);
          可见态新按/换钮立即发一次; 方向键按住每 repeat_ms 连发; dot/save 单发。
    防抖(guard_ms): 与"最近一次发出动作的同一按钮"、自上次动作 < guard_ms 的再次按沿
          视为触摸抖动吞掉(防一次点击跳两项); 换钮不受限; guard_ms=0 禁用。
          基准用 act_t(最近动作时刻)而非 hold_t0 —— hold_t0 还承担连发计时, 语义已分家。
    action ∈ None / "up" / "down" / "left" / "right" / "dot" / "save" / "mode" / "param"
    """
    if not vis:
        if pressed and not was_pressed:
            return True, None, now, last_btn, act_t, None
        return False, None, hold_t0, last_btn, act_t, None
    if not pressed:
        return True, None, hold_t0, last_btn, act_t, None
    if hit is None:
        return True, None, now, last_btn, act_t, None
    if hit != btn:
        if guard_ms > 0 and hit == last_btn \
                and _tdiff(now, act_t) < guard_ms:
            print("touch: guarded %s dt=%d (now=%d act_t=%d)"   # V13 诊断(定位后移除)
                  % (hit, _tdiff(now, act_t), now, act_t))
            return True, hit, hold_t0, last_btn, act_t, None
        print("touch: fire %s now=%d" % (hit, now))   # V13 诊断(定位后移除)
        return True, hit, now, hit, now, hit
    if (hit == "up" or hit == "down" or hit == "left" or hit == "right") \
            and _tdiff(now, hold_t0) >= repeat_ms:
        return True, hit, now, hit, now, hit
    return True, hit, hold_t0, last_btn, act_t, None

def _replace_json_int(line, val):
    """把一行 json 里第一个整数 token 换成 val(保留键/缩进/尾逗号); 无整数 token 返回 None"""
    i = line.find(":")
    if i < 0:
        return None
    j = i + 1
    n = len(line)
    while j < n and line[j] == " ":
        j += 1
    k = j
    if k < n and line[k] == "-":
        k += 1
    s = k
    while k < n and "0" <= line[k] <= "9":
        k += 1
    if k == s:
        return None
    return line[:j] + str(val) + line[k:]

def calib_save_text(src, lox, loy):
    """config.json 文本 → 替换 laser_offset_x/y 两行的整数值; 其余行逐字节保留。

    成功返回新文本; 两个键未同时找到返回 None(调用方按失败处理, 不写盘)。"""
    lines = src.split("\n")
    hit = 0
    for i in range(len(lines)):
        s = lines[i].strip()
        if s.startswith('"laser_offset_x"') or s.startswith('"laser_offset_y"'):
            val = lox if s.startswith('"laser_offset_x"') else loy
            nl = _replace_json_int(lines[i], val)
            if nl is None:
                return None
            lines[i] = nl
            hit += 1
    if hit != 2:
        return None
    return "\n".join(lines)

def calib_save_file(path, lox, loy):
    """读 config → 文本替换 → 备份(.bak) → 覆盖写 → 读回校验。返回 (ok, msg)。

    任何失败不动原文件(备份为上一版)。无 path(未从文件加载配置)时直接失败。"""
    if not path:
        return False, "no config path"
    try:
        with open(path, "r") as f:
            src = f.read()
        dst = calib_save_text(src, lox, loy)
        if dst is None:
            return False, "laser_offset keys not found"
        try:
            with open(path + ".bak", "w") as f:
                f.write(src)
        except Exception:
            pass
        with open(path, "w") as f:
            f.write(dst)
        try:
            import ujson as _j
        except ImportError:
            import json as _j
        with open(path, "r") as f:
            _j.load(f)
        return True, "saved (%d,%d)" % (lox, loy)
    except Exception as e:
        return False, str(e)

# ---- V7.9 触屏调参: 数值格式化 / 参数表 / 步进逻辑(纯函数, 主机可测) ----

def fmt_num(v):
    """数值 → JSON 文本: int 原样; float 最多 4 位小数去尾零(0.2800→0.28, 1.0→1)"""
    if isinstance(v, int):
        return str(v)
    s = "%.4f" % v
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s

def fmt_disp(item, val):
    """触屏显示值: 开关项 → ON/OFF; 浮点 2 位小数; 整数原样"""
    if item == "pid" or item == "att":
        return "ON" if val else "OFF"
    if isinstance(val, float):
        return "%.2f" % val
    return "%d" % val

# 触屏可调的 12 项(循环顺序; "pid"/"att"=开关, 其余为数值)
# V15: 末尾三项(att/akp/adz)只对 yaw 轴生效 —— 触屏切到 pitch 轴时会自动跳过。
PARAM_ITEMS = ("pid", "kp", "ki", "kd", "max", "min", "dead", "acc", "full",
               "att", "akp", "adz")
PARAM_ATT_ITEMS = ("att", "akp", "adz")
# 按钮/显示用短名
PARAM_NAMES = {"pid": "PID", "kp": "Kp", "ki": "Ki", "kd": "Kd",
               "max": "Max", "min": "Min", "dead": "Dead", "acc": "Acc",
               "full": "Full", "att": "ATT", "akp": "AKp", "adz": "ADz"}
# 数值项步长/范围: (步长, 下限, 上限, 是否浮点)
#   akp = 姿态角度环 P [(度/秒)/度];  adz = 姿态速率死区 (°/s)
PARAM_SPEC = {
    "kp":   (0.02, 0.0, 5.0, True),
    "ki":   (0.01, 0.0, 2.0, True),
    "kd":   (0.01, 0.0, 2.0, True),
    "max":  (1, 1, 500, False),
    "min":  (1, 0, 500, False),
    "dead": (1, 0, 100, False),
    "acc":  (5, 0, 255, False),
    "full": (10, 20, 640, False),
    "akp":  (1.0, 1.0, 60.0, True),
    "adz":  (0.5, 0.0, 30.0, True),
}

def param_next(item, axis=0):
    """参数项循环: 返回 item 的下一项(表尾回卷); 未知项回到第一项。

    V15: axis=1(pitch) 时跳过姿态三项(它们只对 yaw 有效), 免得 p 轴上出现死键。
    """
    n = len(PARAM_ITEMS)
    for i in range(n):
        if PARAM_ITEMS[i] == item:
            j = (i + 1) % n
            while axis != 0 and PARAM_ITEMS[j] in PARAM_ATT_ITEMS:
                j = (j + 1) % n
            return PARAM_ITEMS[j]
    return PARAM_ITEMS[0]

def param_calc(item, val, direction):
    """调参步进(纯函数): item 当前值 val 按 direction(+1/-1) 走一格。

    - 数值项: ±步长, clamp 到 [下限,上限], 浮点四舍五入到 4 位小数(防尾数漂移)
    - 开关项("pid"/"att"): +1 → 1(开), -1 → 0(关)
    """
    if item == "pid" or item == "att":
        return 1 if direction > 0 else 0
    step, lo, hi, isf = PARAM_SPEC[item]
    v = val + (step if direction > 0 else -step)
    if v < lo:
        v = lo
    elif v > hi:
        v = hi
    if isf:
        return round(v, 4)
    return int(v)

def param_value(axis, item):
    """读触屏调参当前值(axis: 0=yaw / 1=pitch)"""
    if axis == 0:
        if item == "kp":   return M_PID_KP
        if item == "ki":   return M_PID_KI
        if item == "kd":   return M_PID_KD
        if item == "max":  return M_MAX_RPM
        if item == "min":  return M_MIN_RPM
        if item == "dead": return M_DEADBAND
        if item == "acc":  return M_ACC
        if item == "full": return M_DX_FULL
        if item == "pid":  return 1 if M_PID_EN else 0
        if item == "att":  return 1 if ATT_EN else 0
        if item == "akp":  return ATT_ANGLE_KP
        if item == "adz":  return ATT_DEADBAND_DPS
        return 0
    if item == "kp":   return P_PID_KP
    if item == "ki":   return P_PID_KI
    if item == "kd":   return P_PID_KD
    if item == "max":  return P_MAX_RPM
    if item == "min":  return P_MIN_RPM
    if item == "dead": return P_DEADBAND
    if item == "acc":  return P_ACC
    if item == "full": return P_DY_FULL
    if item == "pid":  return 1 if P_PID_EN else 0
    return 0

def param_apply(axis, item, direction):
    """调参步进并写回运行时全局(gimbal_send 下一帧即生效); "acc" 同步进电机实例。返回新值。"""
    global M_PID_EN, M_PID_KP, M_PID_KI, M_PID_KD
    global M_MAX_RPM, M_MIN_RPM, M_DEADBAND, M_ACC, M_DX_FULL
    global P_PID_EN, P_PID_KP, P_PID_KI, P_PID_KD
    global P_MAX_RPM, P_MIN_RPM, P_DEADBAND, P_ACC, P_DY_FULL
    global ATT_ANGLE_KP, ATT_DEADBAND_DPS
    if axis == 0:
        if item == "kp":
            M_PID_KP = param_calc(item, M_PID_KP, direction); return M_PID_KP
        if item == "ki":
            M_PID_KI = param_calc(item, M_PID_KI, direction); return M_PID_KI
        if item == "kd":
            M_PID_KD = param_calc(item, M_PID_KD, direction); return M_PID_KD
        if item == "max":
            M_MAX_RPM = param_calc(item, M_MAX_RPM, direction); return M_MAX_RPM
        if item == "min":
            M_MIN_RPM = param_calc(item, M_MIN_RPM, direction); return M_MIN_RPM
        if item == "dead":
            M_DEADBAND = param_calc(item, M_DEADBAND, direction); return M_DEADBAND
        if item == "acc":
            M_ACC = param_calc(item, M_ACC, direction)
            if _yaw:
                _yaw.acc = M_ACC        # 已构造实例持有副本, 同步后新值才生效
            return M_ACC
        if item == "full":
            M_DX_FULL = param_calc(item, M_DX_FULL, direction); return M_DX_FULL
        if item == "pid":
            M_PID_EN = (param_calc(item, 0, direction) == 1)
            return 1 if M_PID_EN else 0
        if item == "att":
            # V15: 姿态开关 —— 走 _att_bind, 顺带重绑 yaw 实现(热路径不留分支)
            _att_bind(param_calc(item, 1 if ATT_EN else 0, direction) == 1)
            return 1 if ATT_EN else 0
        if item == "akp":
            ATT_ANGLE_KP = param_calc(item, ATT_ANGLE_KP, direction); return ATT_ANGLE_KP
        if item == "adz":
            ATT_DEADBAND_DPS = param_calc(item, ATT_DEADBAND_DPS, direction)
            return ATT_DEADBAND_DPS
        return 0
    if item == "kp":
        P_PID_KP = param_calc(item, P_PID_KP, direction); return P_PID_KP
    if item == "ki":
        P_PID_KI = param_calc(item, P_PID_KI, direction); return P_PID_KI
    if item == "kd":
        P_PID_KD = param_calc(item, P_PID_KD, direction); return P_PID_KD
    if item == "max":
        P_MAX_RPM = param_calc(item, P_MAX_RPM, direction); return P_MAX_RPM
    if item == "min":
        P_MIN_RPM = param_calc(item, P_MIN_RPM, direction); return P_MIN_RPM
    if item == "dead":
        P_DEADBAND = param_calc(item, P_DEADBAND, direction); return P_DEADBAND
    if item == "acc":
        P_ACC = param_calc(item, P_ACC, direction)
        if _pitch:
            _pitch.acc = P_ACC
        return P_ACC
    if item == "full":
        P_DY_FULL = param_calc(item, P_DY_FULL, direction); return P_DY_FULL
    if item == "pid":
        P_PID_EN = (param_calc(item, 0, direction) == 1)
        return 1 if P_PID_EN else 0
    return 0

def param_save_items():
    """收集写回 config 的全部触屏调参项: [(段名, 键名, 文本值), ...] (调参页 S 键用)"""
    return (
        ("motor", "pid_enabled", "true" if M_PID_EN else "false"),
        ("motor", "pid_kp", fmt_num(M_PID_KP)),
        ("motor", "pid_ki", fmt_num(M_PID_KI)),
        ("motor", "pid_kd", fmt_num(M_PID_KD)),
        ("motor", "min_rpm", fmt_num(M_MIN_RPM)),
        ("motor", "max_rpm", fmt_num(M_MAX_RPM)),
        ("motor", "dx_full_px", fmt_num(M_DX_FULL)),
        ("motor", "deadband_px", fmt_num(M_DEADBAND)),
        ("motor", "acc", fmt_num(M_ACC)),
        ("pitch_motor", "pid_enabled", "true" if P_PID_EN else "false"),
        ("pitch_motor", "pid_kp", fmt_num(P_PID_KP)),
        ("pitch_motor", "pid_ki", fmt_num(P_PID_KI)),
        ("pitch_motor", "pid_kd", fmt_num(P_PID_KD)),
        ("pitch_motor", "min_rpm", fmt_num(P_MIN_RPM)),
        ("pitch_motor", "max_rpm", fmt_num(P_MAX_RPM)),
        ("pitch_motor", "dy_full_px", fmt_num(P_DY_FULL)),
        ("pitch_motor", "deadband_px", fmt_num(P_DEADBAND)),
        ("pitch_motor", "acc", fmt_num(P_ACC)),
        # V15: 姿态三项(只对 yaw 有效)
        ("attitude", "enabled", "true" if ATT_EN else "false"),
        ("attitude", "angle_kp", fmt_num(ATT_ANGLE_KP)),
        ("attitude", "deadband_dps", fmt_num(ATT_DEADBAND_DPS)),
    )

def _replace_json_token(line, text):
    """把一行 json 里冒号后第一个 token 换成 text(保留键/缩进/逗号等其余文本);
    无 token(空值)返回 None。可替换数字 / true / false 等任意标量 token。"""
    i = line.find(":")
    if i < 0:
        return None
    j = i + 1
    n = len(line)
    while j < n and line[j] == " ":
        j += 1
    k = j
    while k < n and line[k] != "," and line[k] != "}" and line[k] != "]" and line[k] != " ":
        k += 1
    if k == j:
        return None
    return line[:j] + text + line[k:]

def param_save_text(src, items):
    """config.json 文本 → 逐项替换 [(段, 键, 文本), ...]; 其余行逐字节保留。

    段跟踪: 顶层段行 = 2 空格缩进的 "name": {; 2 空格缩进的 } / }, 关段。
    仅替换"位于对应段内"的键行; 任一项未命中 → 返回 None(调用方按失败处理, 不写盘)。
    """
    lines = src.split("\n")
    todo = list(items)
    cur = None
    for i in range(len(lines)):
        s = lines[i].strip()
        ind = len(lines[i]) - len(lines[i].lstrip(" "))
        if ind == 2 and s.startswith('"') and s.rstrip().endswith("{"):
            q = s.find('"', 1)
            cur = s[1:q] if q > 0 else None
            continue
        if ind == 2 and (s == "}" or s == "},"):
            cur = None
            continue
        if cur is None or not s.startswith('"') or ":" not in s:
            continue
        q = s.find('"', 1)
        if q < 0:
            continue
        key = s[1:q]
        for t in range(len(todo)):
            sec, k, text = todo[t]
            if sec == cur and k == key:
                nl = _replace_json_token(lines[i], text)
                if nl is None:
                    return None
                lines[i] = nl
                del todo[t]
                break
    if todo:
        return None
    return "\n".join(lines)

def param_save_file(path, items):
    """读 config → 逐项文本替换 → 备份(.bak) → 覆盖写 → 读回校验(合法 JSON + 逐项值吻合)。
    返回 (ok, msg); 任何失败不动原文件。无 path(未从文件加载配置)时直接失败。"""
    if not path:
        return False, "no config path"
    try:
        with open(path, "r") as f:
            src = f.read()
        dst = param_save_text(src, items)
        if dst is None:
            return False, "keys not found (config 缺新键?)"
        try:
            with open(path + ".bak", "w") as f:
                f.write(src)
        except Exception:
            pass
        with open(path, "w") as f:
            f.write(dst)
        try:
            import ujson as _j
        except ImportError:
            import json as _j
        with open(path, "r") as f:
            cfg = _j.load(f)
        for sec, key, text in items:
            try:
                got = cfg[sec][key]
            except Exception:
                return False, "verify fail: %s.%s" % (sec, key)
            if text == "true" or text == "false":
                okv = (bool(got) == (text == "true"))
            else:
                try:
                    okv = abs(float(got) - float(text)) < 1e-6
                except Exception:
                    okv = False
            if not okv:
                return False, "verify fail: %s.%s" % (sec, key)
        return True, "saved %d keys" % len(items)
    except Exception as e:
        return False, str(e)

def param_step(vis, btn, hold_t0, last_btn, now, pressed, was_pressed, hit, repeat_ms, guard_ms, act_t):
    """调参页触摸状态机(规则与 calib_step 相同; 连发按钮 = plus/minus)。
    返回 (vis, btn, hold_t0, last_btn, act_t, action);
    防抖(guard_ms): 同钮自上次动作 < guard_ms 的再次按沿被吞(防"一下跳两项"); 换钮不受限; 0=禁用。
    action ∈ None / "plus" / "minus" / "item" / "axis" / "save" / "back"
    """
    if not vis:
        if pressed and not was_pressed:
            return True, None, now, last_btn, act_t, None
        return False, None, hold_t0, last_btn, act_t, None
    if not pressed:
        return True, None, hold_t0, last_btn, act_t, None
    if hit is None:
        return True, None, now, last_btn, act_t, None
    if hit != btn:
        if guard_ms > 0 and hit == last_btn \
                and _tdiff(now, act_t) < guard_ms:
            print("touch: guarded %s dt=%d (now=%d act_t=%d)"   # V13 诊断(定位后移除)
                  % (hit, _tdiff(now, act_t), now, act_t))
            return True, hit, hold_t0, last_btn, act_t, None
        print("touch: fire %s now=%d" % (hit, now))   # V13 诊断(定位后移除)
        return True, hit, now, hit, now, hit
    if (hit == "plus" or hit == "minus") \
            and _tdiff(now, hold_t0) >= repeat_ms:
        return True, hit, now, hit, now, hit
    return True, hit, hold_t0, last_btn, act_t, None

def _param_rects(ox, oy, sw, sh, bw):
    """生成调参页 6 个按钮的命中矩形(LCD 坐标): +/− (左缘上下)
    项(PID/Kp/...) · 轴(Y/P) (右缘上下) · S保存 · X返回(顶中)"""
    m = 6
    y1 = sh // 2 - bw - 6
    y2 = y1 + bw + 12
    xl = ox + m
    xr = ox + sw - m - bw
    xc = ox + sw // 2
    return (
        ("plus",  xl, oy + y1, xl + bw, oy + y1 + bw),
        ("minus", xl, oy + y2, xl + bw, oy + y2 + bw),
        ("item",  xr, oy + y1, xr + bw, oy + y1 + bw),
        ("axis",  xr, oy + y2, xr + bw, oy + y2 + bw),
        ("save",  xc - bw - 4, oy + m, xc - 4, oy + m + bw),
        ("back",  xc + 4, oy + m, xc + bw + 4, oy + m + bw),
    )

def param_draw(frame_np, rects, axis, item, val, pid_on, circle_on=False):
    """绘制调参页: 6 按钮 + 底部当前值行("<Yaw/Pitch> <项> = <值>  PID:<ON/OFF>  C:<ON/OFF>")。
    circle_on: 当前画圈模式状态(屏上显示, 便于在任何页知道画圈开没开)。
    V11: rects 与触摸命中同为图像坐标系(resize_map_pos_reverse 已把屏幕触点映射回图像系)。"""
    for name, x0, y0, x1, y1 in rects:
        a0, b0, a1, b1 = x0, y0, x1, y1
        cv2.line(frame_np, (a0, b0), (a1, b0), (0, 200, 200), 2)
        cv2.line(frame_np, (a1, b0), (a1, b1), (0, 200, 200), 2)
        cv2.line(frame_np, (a1, b1), (a0, b1), (0, 200, 200), 2)
        cv2.line(frame_np, (a0, b1), (a0, b0), (0, 200, 200), 2)
        mx = (a0 + a1) // 2
        my = (b0 + b1) // 2
        if name == "plus":
            cv2.line(frame_np, (mx - 12, my), (mx + 12, my), (0, 200, 200), 3)
            cv2.line(frame_np, (mx, my - 12), (mx, my + 12), (0, 200, 200), 3)
        elif name == "minus":
            cv2.line(frame_np, (mx - 12, my), (mx + 12, my), (0, 200, 200), 3)
        elif name == "item":
            cv2.putText(frame_np, PARAM_NAMES.get(item, item), (mx - 14, my + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 200), 2)
        elif name == "axis":
            cv2.putText(frame_np, "P" if axis else "Y", (mx - 8, my + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 200), 2)
        elif name == "save":
            cv2.putText(frame_np, "S", (mx - 8, my + 9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 200), 2)
        else:
            cv2.putText(frame_np, "X", (mx - 8, my + 9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 200), 2)
    vt = "%s %s = %s   PID:%s  C:%s" % (
        "Pitch" if axis else "Yaw", PARAM_NAMES.get(item, item),
        fmt_disp(item, val), "ON" if pid_on else "OFF",
        "ON" if circle_on else "OFF")
    cv2.putText(frame_np, vt, ((SENSOR_W - 8 * len(vt)) // 2, SENSOR_H - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

def _calib_arrow(frame_np, cx, cy, name):
    """方向按钮符号: 三角描边(上/下/左/右)"""
    s = 13
    if name == "up":
        p = ((cx, cy - s), (cx - s, cy + s // 2), (cx + s, cy + s // 2))
    elif name == "down":
        p = ((cx, cy + s), (cx - s, cy - s // 2), (cx + s, cy - s // 2))
    elif name == "left":
        p = ((cx - s, cy), (cx + s // 2, cy - s), (cx + s // 2, cy + s))
    else:
        p = ((cx + s, cy), (cx - s // 2, cy - s), (cx - s // 2, cy + s))
    cv2.line(frame_np, p[0], p[1], (0, 200, 0), 2)
    cv2.line(frame_np, p[1], p[2], (0, 200, 0), 2)
    cv2.line(frame_np, p[2], p[0], (0, 200, 0), 2)

def calib_draw(frame_np, rects, lox, loy, dot_r, mode_on=False):
    """绘制校准 UI: 按钮(框+符号, 含追踪/画圈切换) + 绿色校准点(画面坐标 CENTER+offset)。
    V11: rects 与触摸命中同为图像坐标系(resize_map_pos_reverse 已把屏幕触点映射回图像系)。"""
    for name, x0, y0, x1, y1 in rects:
        a0, b0, a1, b1 = x0, y0, x1, y1
        cv2.line(frame_np, (a0, b0), (a1, b0), (0, 140, 0), 2)
        cv2.line(frame_np, (a1, b0), (a1, b1), (0, 140, 0), 2)
        cv2.line(frame_np, (a1, b1), (a0, b1), (0, 140, 0), 2)
        cv2.line(frame_np, (a0, b1), (a0, b0), (0, 140, 0), 2)
        mx = (a0 + a1) // 2
        my = (b0 + b1) // 2
        if name == "dot":
            cv2.circle(frame_np, (mx, my), 12, (0, 200, 0), 2)
            cv2.circle(frame_np, (mx, my), 3, (0, 200, 0), -1)
        elif name == "save":
            cv2.putText(frame_np, "S", (mx - 8, my + 9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)
        elif name == "mode":
            # 追踪/画圈切换: 当前模式 C=画圈(橙) / T=追踪(绿)
            col = (0, 165, 255) if mode_on else (0, 200, 0)
            cv2.putText(frame_np, "C" if mode_on else "T", (mx - 9, my + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
        elif name == "param":
            # 进入调参页(PID 与云台参数)
            cv2.putText(frame_np, "P", (mx - 8, my + 9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)
        else:
            _calib_arrow(frame_np, mx, my, name)
    gx = CENTER_X + lox
    gy = CENTER_Y + loy
    cv2.circle(frame_np, (gx, gy), dot_r, (0, 255, 0), -1)
    cv2.circle(frame_np, (gx, gy), dot_r + 1, (255, 255, 255), 1)

# V11: 按钮直接生成在图像坐标系(0,0 起点); 屏幕触摸经 resize_map_pos_reverse 映射回该系后命中
CALIB_BTNS = _calib_rects(0, 0, SENSOR_W, SENSOR_H, 56)
PARAM_BTNS = _param_rects(0, 0, SENSOR_W, SENSOR_H, 56)

# ============================================================================
# 七、CV 模式主循环
# ============================================================================

def run_cv():
    global LOX, LOY                  # 触屏中心校准运行时更新激光偏移
    cam = None
    frame_id = lost_count = 0
    last_box = None
    _circle_on = False               # 画圈模式(模式引脚去抖后)
    _pin_streak = 0
    _theta = None                    # 画圈 θ(None=未起始; 见框时按当前方位角起转)
    _theta_wait = 0                  # 画圈连续未到位累计(ms); 到位清零, 超 CIRC_WAIT_MAX 强制推进
    _theta_tick = time.ticks_ms()    # θ 推进的帧间计时基准
    _circ_log_tick = time.ticks_ms() # 画圈诊断日志限频
    last_dist = 0.0                  # 上帧瞄准偏差(画圈误差门控用)
    _calib_tp = None                 # 触摸句柄(None=不可用/未启用)
    _calib_vis = CALIB_SHOW_DOT      # 绿点+按钮 显隐(dot按钮切换; 隐藏后触摸任意处唤出)
    _calib_btn = None                # 按住中的按钮名
    _calib_hold = 0                  # 按住起始时刻(连发计时)
    _calib_act_t = 0                 # 最近一次动作发出时刻(防抖基准; 0=尚无动作)
    _calib_pre = False               # 上帧是否有触点(唤醒边沿判定)
    _ts_evt_tick = time.ticks_ms()   # 最近一次触摸底层事件时刻(静止看门狗)
    _ts_stale_on = False             # 看门狗强制抬手状态(防恢复日志刷屏)
    _touch_err_n = 0                 # read 连续异常计数(限频打印; 不再永久禁用触摸)
    _calib_last = None               # 最近一次发出动作的按钮(防抖基准; 防"一下跳两项")
    _page = 0                        # 触屏页: 0=校准 / 1=调参(P 键进入, X 键返回)
    _paxis = 0                       # 调参页当前轴(0=yaw / 1=pitch; 轴键切换)
    _pitem = "pid"                   # 调参页当前项(项键循环; 见 PARAM_ITEMS)
    _calib_skip = False              # 切页后吞掉同一次按压的残留动作(防跨页串按)
    _mon_tick_y = time.ticks_ms()    # V8: yaw 堵转监控计时(monitor_ms>0 时用)
    _mon_tick_p = time.ticks_ms()    # V8: pitch 堵转监控计时
    fps = fps_count = 0
    fps_tick = time.ticks_ms()

    try:
        print("boot")
        # MaixCAM2 采集: Camera 直接出目标尺寸 BGR888(供 cv2 零拷贝处理);
        # 宽高比与传感器(16:9)不一致时居中裁切; ≤1280x720 可到 60/80fps
        cam = camera.Camera(SENSOR_W, SENSOR_H, image.Format.FMT_BGR888, fps=FPS)
        cam.skip_frames(10)          # 跳过开头不稳定帧
        print("sensor ok")
        disp = display.Display()
        _dw = disp.width()           # 实际屏幕分辨率(MaixCAM2 自带 640x480 触摸屏)
        _dh = disp.height()
        # 启动几何摘要: 串口一眼核对 采集/显示/中心 是否与期望一致(防拷错 config)
        print("geom: cam=%dx%d lcd=%dx%d center=(%d,%d)"
              % (SENSOR_W, SENSOR_H, _dw, _dh, CENTER_X, CENTER_Y))
        # ---- 触屏中心校准: 触摸初始化(失败自动禁用, 不影响主程序) ----
        if CALIB_ENABLED:
            try:
                _calib_tp = touchscreen.TouchScreen()
                print("calib: touch ok")
            except Exception as _e:
                print("calib: touch unavailable (%s) -- disabled" % _e)
                _calib_tp = None
        print("run ok")
        # V13/V14 诊断: 画圈配置快照(核对板上 config 与 PC 是否一致, 防"跑的是旧配置")
        print("circ cfg: speed=%.1f dir=%d pause=%d wait_max=%d bias=%.1fcm rect=%.1fx%.1f dia=%.1f persp=on"
              % (CIRC_SPEED, CIRC_DIR, CIRC_PAUSE, CIRC_WAIT_MAX, RADIUS_BIAS_CM,
                 RECT_W_CM, RECT_H_CM, CIRCLE_DIA_CM))
        # V15 诊断: 姿态配置快照(同样是"版本指纹", 没有这行=板上跑的不是 V15 代码)
        print("att cfg: on=%s ppd=%.2f tick=%dus budget=%dms sign=%+d akp=%.1f ki=%.2f "
              "dead=%.1f hold=%dms fault=%.0f turns=%d @%.0f°/s"
              % ("Y" if ATT_EN else "N", _att_ppd, ATT_TICK_US, ATT_BUDGET_MS,
                 ATT_YAW_SIGN, ATT_ANGLE_KP, ATT_RATE_KI, ATT_DEADBAND_DPS,
                 ATT_HOLD_MS, ATT_FAULT_DEG, SEARCH_TURNS, SEARCH_DPS))
        # 姿态接管时开机就进搜索(与 gimbal_send 的 search 分支等价, 这里先起起来省一帧)
        if ATT_EN and SEARCH_EN and not _acquired:
            _search_yaw_att()

        while not app.need_exit():
            # V15: 视觉帧间跑姿态内环(放在 cam.read 之前 —— 此刻相机正在缓冲下一帧,
            #      这段时间本来就在等, 所以内环不额外吃掉视觉帧率)。姿态关闭时是空操作。
            _att_spin(ATT_BUDGET_MS)
            frame_id += 1
            img = cam.read()
            frame_np = image.image2cv(img, ensure_bgr=False, copy=False)  # BGR 零拷贝(cv2 直改帧存)

            # ---- V7.7/7.9 触屏: 校准页(移动绿点/显隐/保存) + 调参页(PID 与云台参数) ----
            if _calib_tp is not None:
                _now_t = time.ticks_ms()
                try:
                    # MaixPy 触摸是事件驱动(内部缓冲; 无事件时 read() 返回上一缓存态)。
                    # 视觉帧间隔较长时, 一次"按下-松开-再按下"可能被压在同一帧, 单次
                    # read() 只看到连续 True、丢掉中间松开沿 -> 表现为按钮只能点一次。
                    # 故每帧 read() 后用 available()/read0() 排空积压事件, 并捕捉本批
                    # 事件中"松开后重新按下"的缝隙, 给状态机补一个按下沿。
                    try:
                        _have_evt = _calib_tp.available(0)
                    except Exception:
                        _have_evt = True            # available 不可查时退回旧行为(照常 read)
                    _tx, _ty, _pressed = _calib_tp.read()
                    _raw_xy = (_tx, _ty)             # V13 诊断: 屏系原始终点(映射前)
                    _touch_err_n = 0
                    _gap = False                    # 本批事件出现过松开/重新按下
                    if _have_evt:
                        _ts_evt_tick = _now_t
                    _drained = 0
                    while _drained < 64:
                        try:
                            if not _calib_tp.available(0):
                                break
                            _nx, _ny, _np = _calib_tp.read0()
                        except Exception:
                            break
                        _drained += 1
                        _ts_evt_tick = _now_t
                        if _np:
                            if not _pressed:
                                _gap = True         # False->True: 一次新按压
                            _pressed = True
                            _tx, _ty = _nx, _ny
                        else:
                            _pressed = False
                            _gap = True
                    # 静止看门狗: 持续按住却长时间无任何事件(正常按住手指微动会有 MOVE),
                    # 视为固件丢了松开事件、状态卡高 -> 本帧强制抬手, 等下一次真 PRESS
                    if _pressed and TOUCH_STALE_MS > 0 \
                            and _tdiff(_now_t, _ts_evt_tick) > TOUCH_STALE_MS:
                        if not _ts_stale_on:
                            print("touch: no event for %dms -> force release"
                                  % _tdiff(_now_t, _ts_evt_tick))
                            _ts_stale_on = True
                        _pressed = False
                        _gap = False
                    elif not _pressed:
                        _ts_stale_on = False
                except Exception as _e:
                    # 偶发 read 异常不再永久禁用触摸(旧逻辑一次异常后整块触屏报废)
                    _touch_err_n += 1
                    if _touch_err_n == 1 or _touch_err_n % 120 == 0:
                        print("calib: touch read fail (%s)" % _e)
                    _tx, _ty, _pressed = 0, 0, False
                    _raw_xy = (0, 0)
                    _drained = 0
                    _gap = False
                if _pressed:
                    # 屏幕坐标 → 图像坐标(disp.show 默认 FIT_CONTAIN 缩放居中, 触点映射回图像系)
                    _tx, _ty = image.resize_map_pos_reverse(
                        SENSOR_W, SENSOR_H, _dw, _dh, image.Fit.FIT_CONTAIN, _tx, _ty)
                # 缝隙补沿: 本批事件含"松开后再按下"时, 把上帧状态按已松开喂入, 强制出 F->T 沿
                _was = False if (_pressed and _gap) else _calib_pre
                # V13 诊断(定位后移除): 按下/松开沿打印 —— 连点排查核心数据
                if _pressed and not _was:
                    _h0 = calib_hit(_tx, _ty, CALIB_BTNS if _page == 0 else PARAM_BTNS)
                    print("touch: DOWN raw=%s img=(%d,%d) page=%d hit=%s drained=%d gap=%s"
                          % (str(_raw_xy), _tx, _ty, _page, _h0, _drained, _gap))
                elif not _pressed and _was:
                    print("touch: UP (drained=%d)" % _drained)
                if _page == 0:
                    _hit = calib_hit(_tx, _ty, CALIB_BTNS) if _pressed else None
                    _calib_vis, _calib_btn, _calib_hold, _calib_last, _calib_act_t, _act = calib_step(
                        _calib_vis, _calib_btn, _calib_hold, _calib_last, _now_t,
                        _pressed, _was, _hit, CALIB_REPEAT, CALIB_DEBOUNCE, _calib_act_t)
                else:
                    _hit = calib_hit(_tx, _ty, PARAM_BTNS) if _pressed else None
                    _calib_vis, _calib_btn, _calib_hold, _calib_last, _calib_act_t, _act = param_step(
                        _calib_vis, _calib_btn, _calib_hold, _calib_last, _now_t,
                        _pressed, _was, _hit, CALIB_REPEAT, CALIB_DEBOUNCE, _calib_act_t)
                _calib_pre = _pressed
                if _calib_skip:              # 切页后的同一次按压: 吞掉动作(松开后恢复)
                    _act = None
                    if not _pressed:
                        _calib_skip = False
                if _page == 0:
                    if _act == "up":
                        LOY -= CALIB_STEP
                    elif _act == "down":
                        LOY += CALIB_STEP
                    elif _act == "left":
                        LOX -= CALIB_STEP
                    elif _act == "right":
                        LOX += CALIB_STEP
                    elif _act == "dot":
                        _calib_vis = False
                    elif _act == "mode":
                        if _mode_pin is None:        # 引脚启用时以引脚为准(避免双源打架)
                            _circle_on = not _circle_on
                            _theta = None            # 进出画圈沿重置起始角
                            _theta_wait = 0
                            print("MODE->%s (touch)" % ("circle" if _circle_on else "track"))
                    elif _act == "save":
                        _ok, _msg = calib_save_file(CFG_PATH, LOX, LOY)
                        print("calib: save %s %s" % ("OK" if _ok else "FAIL", _msg))
                    elif _act == "param":
                        if SERVO_ENABLED:
                            print("servo: 触屏调参页暂未映射 -> 改 config.json [servo]")
                        else:
                            _page = 1
                            _calib_btn = None
                            _calib_hold = 0
                            _calib_act_t = 0             # 跨页重置防抖基准
                            _calib_last = None           # 跨页不沿用防抖基准
                            _calib_skip = True           # 防按 P 未松手时误触调参页按钮
                            print("UI->param page")
                    if LOX < -CALIB_XMAX:
                        LOX = -CALIB_XMAX
                    elif LOX > CALIB_XMAX:
                        LOX = CALIB_XMAX
                    if LOY < -CALIB_YMAX:
                        LOY = -CALIB_YMAX
                    elif LOY > CALIB_YMAX:
                        LOY = CALIB_YMAX
                else:
                    if _act == "plus":
                        _v = param_apply(_paxis, _pitem, 1)
                        print("tune: %s %s=%s" % ("pitch" if _paxis else "yaw",
                                                  _pitem, fmt_num(_v)))
                    elif _act == "minus":
                        _v = param_apply(_paxis, _pitem, -1)
                        print("tune: %s %s=%s" % ("pitch" if _paxis else "yaw",
                                                  _pitem, fmt_num(_v)))
                    elif _act == "item":
                        _pitem = param_next(_pitem, _paxis)   # pitch 轴跳过姿态三项
                        print("tune: item -> %s" % _pitem)
                    elif _act == "axis":
                        _paxis = 1 - _paxis
                        if _paxis == 1 and _pitem in PARAM_ATT_ITEMS:
                            _pitem = "pid"                     # 切到 pitch 时离开姿态项
                        print("tune: axis -> %s" % ("pitch" if _paxis else "yaw"))
                    elif _act == "save":
                        _ok, _msg = param_save_file(CFG_PATH, param_save_items())
                        print("param: save %s %s" % ("OK" if _ok else "FAIL", _msg))
                    elif _act == "back":
                        _page = 0
                        _calib_btn = None
                        _calib_hold = 0
                        _calib_act_t = 0             # 跨页重置防抖基准
                        _calib_last = None           # 跨页不沿用防抖基准
                        _calib_skip = True
                        print("UI->calib page")

            # ---- V7 模式引脚采样 + 去抖: 被供电=画圈, 其余=仅追踪 ----------------
            if _mode_pin is not None:
                _raw = _mode_pin.value()
                _raw_on = ((_raw != 0) if PIN_ACTIVE == 1 else (_raw == 0))
                _prev_on = _circle_on
                _circle_on, _pin_streak = mode_pin_step(_circle_on, _pin_streak,
                                                        _raw_on, PIN_DEBOUNCE)
                if _circle_on != _prev_on:
                    _theta = None                    # 进出画圈沿重置起始角
                    _theta_wait = 0
                    print("MODE->%s" % ("circle" if _circle_on else "track"))

            # ---- 检测(每检测帧): 矩形 ----------------
            if frame_id % DETECT_EVERY == 0 or not last_box:
                box, contour_count, area_count = find_paper_box(frame_np, last_box)
            else:
                box = last_box

            # ---- 目标选择: 画圈(纯几何) > 追踪(矩形中心) ------------------------
            now2 = time.ticks_ms()
            dtm = _tdiff(now2, _theta_tick)
            _theta_tick = now2
            kind, tx, ty = "none", 0, 0
            dist_mm = 0.0                    # V9: 面积法估算距离(仅矩形追踪路径计算)
            laser_pt = None                  # V9: 激光映射点[(x,y); 未算出=None]
            _circle_view = None              # V14: 画圈圆可视化(折线 Nx1x2: 透视/正圆)
            if _circle_on:
                # V14 透视圆: 矩形4角点(真实透视) ↔ 物理矩形建单应, 物理圆投影到图像,
                # 橙圆/黄点均随矩形透视变形; 单应退化时降级为正圆几何推算(原 V10 行为)
                _persp = circle_persp(box) if box is not None else None
                _rr = geo_radius_px(box) if (_persp is None and box is not None) else 0
                if _persp is not None:
                    _circle_pts, _persp_fn, _r_cm = _persp
                    rcx, rcy, _dcx, _dcy = draw_box(frame_np, box)   # 画框 + 圆心
                    _circle_view = _circle_pts       # 透视圆折线(橙) — 与矩形透视一致
                    if _theta is None:
                        _theta = 90.0    # 物理系 y 向下: 90°=靶纸下缘中点起转(θ 语义与正圆版一致)
                    _theta, _theta_wait = circle_theta_step(
                        _theta, CIRC_SPEED, dtm, CIRC_DIR, last_dist, CIRC_PAUSE,
                        _theta_wait, CIRC_WAIT_MAX)
                    _ra = math.radians(_theta)
                    tx, ty = _persp_fn(_r_cm * math.cos(_ra), _r_cm * math.sin(_ra))
                    kind = "geo"
                    # V13 诊断: 500ms 限频打印画圈推进状态
                    #   th   每条应递增 ~30(=60°/s×0.5s); 不增=θ冻结
                    #   tgt  黄点像素坐标(应绕 c 转); c=圆心(=矩形中心, 只随检测抖动小幅晃)
                    if _tdiff(now2, _circ_log_tick) >= 500:
                        _circ_log_tick = now2
                        print("circ: th=%.0f r=%.2fcm err=%.0f wait=%d/%d tgt=(%d,%d) c=(%d,%d)%s"
                              % (_theta, _r_cm, last_dist, _theta_wait, CIRC_WAIT_MAX,
                                 tx, ty, rcx, rcy,
                                 " FROZEN" if (last_dist > CIRC_PAUSE
                                               and _theta_wait < CIRC_WAIT_MAX) else ""))
                elif _rr > 0:
                    # 降级: 单应不可用 -> 正圆几何推算(V10 原路径)
                    rcx, rcy, _dcx, _dcy = draw_box(frame_np, box)   # 画框 + 圆心
                    _circle_view = circle_polyline(rcx, rcy, _rr)    # 正圆折线(橙)
                    if _theta is None:
                        _theta = math.degrees(math.atan2(CENTER_Y - rcy, CENTER_X - rcx)) % 360.0
                    _theta, _theta_wait = circle_theta_step(
                        _theta, CIRC_SPEED, dtm, CIRC_DIR, last_dist, CIRC_PAUSE,
                        _theta_wait, CIRC_WAIT_MAX)
                    tx, ty = circle_target_point(rcx, rcy, _rr, _theta)
                    kind = "geo"
                    if _tdiff(now2, _circ_log_tick) >= 500:
                        _circ_log_tick = now2
                        print("circ(2d): th=%.0f r=%d err=%.0f wait=%d/%d tgt=(%d,%d) c=(%d,%d)%s"
                              % (_theta, _rr, last_dist, _theta_wait, CIRC_WAIT_MAX,
                                 tx, ty, rcx, rcy,
                                 " FROZEN" if (last_dist > CIRC_PAUSE
                                               and _theta_wait < CIRC_WAIT_MAX) else ""))
                elif _tdiff(now2, _circ_log_tick) >= 500:
                    # V13 诊断: 画圈已开启却没跑起来 —— 照亮三种死法(无框/透视退化/半径0)
                    _circ_log_tick = now2
                    print("circ: idle (%s)" % ("no box" if box is None
                                               else ("persp fail" if _rr <= 0 else "r=0")))
            if kind == "none" and box is not None:
                rcx, rcy, _dcx, _dcy = draw_box(frame_np, box)   # 画框 + 矩形中心
                tx, ty = rcx, rcy
                kind = "rect"
                if DIST_EN:                  # V9: 面积法测距 + 激光点映射(仅矩形路径, 面积标定只对靶标成立)
                    dist_mm = estimate_distance_by_area(quad_area(box))
                    laser_pt = laser_position_by_distance(dist_mm)

            # ---- 执行(保持既有 gimbal_send track 语义) ----------------
            if kind != "none":
                lost_count = 0
                if box is not None:
                    last_box = box
                if LASER_AIM and laser_pt is not None and kind == "rect":
                    dx, dy = tx - laser_pt[0], ty - laser_pt[1]   # V9: 收敛点=激光落点(靶心追到激光点上, 激光打中)
                else:
                    dx, dy = tx - CENTER_X - LOX, ty - CENTER_Y - LOY
                dist = math.sqrt(dx * dx + dy * dy)
                last_dist = dist
                gimbal_send(int(dx), int(dy), dist, "track")
                # OSD: 画圈几何圆/最终瞄准点 标记 + 目标源文字
                if SHOW_OSD:
                    if _circle_view is not None:     # V14: 画圈圆(橙, 折线: 透视椭圆或正圆)
                        cv2.polylines(frame_np, [_circle_view], True, (0, 165, 255), 1,
                                      cv2.LINE_AA)
                    cv2.circle(frame_np, (tx, ty), 4, (0, 255, 255), 1)
                    cv2.putText(frame_np, "[%s] dx:%+d dy:%+d" % (kind, dx, dy),
                                (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    if laser_pt is not None:         # V9: 激光映射点(黄圈+红心)与估算距离
                        cv2.circle(frame_np, (laser_pt[0], laser_pt[1]), 10, (0, 255, 255), 2)
                        cv2.circle(frame_np, (laser_pt[0], laser_pt[1]), 3, (255, 0, 0), -1)
                        cv2.putText(frame_np, "D:%dmm" % int(dist_mm), (4, 48),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                if frame_id % PRINT_EVERY == 0:
                    _dsfx = (" D=%.0fmm" % dist_mm) if dist_mm > 0 else ""   # V9: 距离估算
                    print("OK [%s] d=(%d,%d) dist=%.0f%s" % (kind, dx, dy, dist, _dsfx))
            else:
                lost_count += 1
                if lost_count > 2:
                    last_box = None
                    if SEARCH_EN and not _acquired:
                        gimbal_send(0, 0, 0, "search")   # 上电搜索: 未捕获期 yaw 匀速旋转找靶
                    else:
                        gimbal_send(0, 0, 0, "lost")
                    if frame_id % PRINT_EVERY == 0:
                        print("LOST")

            # FPS
            fps_count += 1
            now = time.ticks_ms()
            if _tdiff(now, fps_tick) >= 1000:
                fps = fps_count * 1000 // _tdiff(now, fps_tick)
                fps_count = 0
                fps_tick = now
                # V15 1Hz 姿态状态行(验收就靠这行口径: 看 err 与 th 随底座转动是否守住)
                if _imu is not None:
                    print("ATT th=%+.1f ref=%+.1f err=%+.2f w=%+.0f dps rpm=%+d mode=%s fps=%d"
                          % (_att_th, _att_ref, _att_err, _att_w, int(_att_rpm),
                             _att_mode, fps))
            cv2.putText(frame_np, "FPS:%d" % fps, (4, SENSOR_H - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            # ---- V15 姿态: OSD 一行 θ/误差/转速/模式(只读观测也显示, 便于 P2 取证) ----
            if SHOW_OSD and _imu is not None:
                cv2.putText(frame_np, "ATT %+.1f d%+.2f z%+.0f %s"
                            % (_att_th, _att_err, _att_w, _att_mode),
                            (4, SENSOR_H - 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 255) if ATT_EN else (160, 160, 160), 1)
            # ---- V8 堵转监控(monitor_ms>0 时每 N ms 读实测转速对照命令; 默认关, 仅告警) ----
            if M_MONITOR_MS > 0 and _yaw and _tdiff(now, _mon_tick_y) >= M_MONITOR_MS:
                _mon_tick_y = now
                if abs(_yaw.last_rpm) >= 3:
                    _rpm_m = _yaw.read_velocity(20)
                    if _rpm_m is not None and -1 <= _rpm_m <= 1:
                        print("WARN: yaw possible stall (cmd=%d meas=%d rpm) — 查机械或发0x0E52清堵转"
                              % (_yaw.last_rpm, _rpm_m))
            if P_MONITOR_MS > 0 and _pitch and _tdiff(now, _mon_tick_p) >= P_MONITOR_MS:
                _mon_tick_p = now
                if abs(_pitch.last_rpm) >= 3:
                    _rpm_m = _pitch.read_velocity(20)
                    if _rpm_m is not None and -1 <= _rpm_m <= 1:
                        print("WARN: pitch possible stall (cmd=%d meas=%d rpm) — 查机械或发0x0E52清堵转"
                              % (_pitch.last_rpm, _rpm_m))
            # 触屏 UI(校准页绿点+按钮 / 调参页, 可隐藏)
            if _calib_vis:
                if _page == 0:
                    calib_draw(frame_np, CALIB_BTNS, LOX, LOY, CALIB_DOT_R, _circle_on)
                else:
                    param_draw(frame_np, PARAM_BTNS, _paxis, _pitem,
                               param_value(_paxis, _pitem),
                               M_PID_EN if _paxis == 0 else P_PID_EN,
                               _circle_on)

            disp.show(img)   # V12: 采集=屏幕分辨率, FIT_CONTAIN 缩放比 1.0, 无黑边铺满全屏
            if frame_id % GC_EVERY == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("user stop")
    except BaseException as e:
        print("error:", e)
    finally:
        if cam:
            try:
                cam.close()
            except Exception:
                pass
        print("exit")


# ============================================================================
# 八、YOLO 模式主循环
# ============================================================================

def run_yolo():
    """V11: YOLO 模式待移植。

    K230 的 KPU 链路(nncase_runtime + libs/PipeLine + libs/YOLO + .kmodel)在 MaixCAM2 上
    不可用; MaixCAM2 需改用 maix.nn.YOLOv8/YOLO11 + AX 格式模型(.axmodel/.mud),
    模型需重新转换。当前版本检测到 mode=yolo 时由入口自动回退 cv 模式, 本函数不会被调用。
    """
    print("YOLO: not ported to MaixCAM2 yet (use detection.mode=cv)")


# ============================================================================
# 九、入口 — 模式切换
# ============================================================================

print("=" * 50)
print("mode: %s | servo: %s | pitch_step: %s | yaw_step: %s" %
      (MODE.upper(), SERVO_ENABLED, PITCH_ENABLED, MOTOR_ENABLED))
print("=" * 50)

# 丢靶预判自检(不跑视觉): 模拟左右两侧丢靶 + 中途重捕获, 单独验证续追行为
# 本块先于 test_mode 判断执行: 与 motor/pitch_motor 的 test_mode 不要同时开(同开时本项优先)。
if mv.get("coast_test", False):
    if not (MOTOR_ENABLED and _yaw and _gimbal_ok):
        print(">>> coast_test=true, 但 yaw 电机未启用 / 初始化未成功 → 跳过丢靶预判自检")
    else:
        M_COAST_EN = True    # 测试模式强制启用预判(不受 coast_enabled 影响)
        print(">>> COAST TEST (丢靶预判自检, 不跑视觉; Ctrl+C 退出)")
        print("  预期: 每侧丢靶后 yaw 朝该侧续转约 %dms 再停; 中途'重捕获'应立即停" % M_COAST_MS)
        for _side, _dx in (("右", 250), ("左", -250)):
            print("---- [%s] 丢靶续追 (最后偏差 dx=%+d) ----" % (_side, _dx))
            gimbal_send(_dx, 0, _dx, "track")      # 模拟"最后看到目标"的一帧
            time.sleep_ms(100)
            _c0 = time.ticks_ms()
            while _tdiff(time.ticks_ms(), _c0) < M_COAST_MS + 1200:
                gimbal_send(0, 0, 0, "lost")       # 连续丢靶 → 观察 COAST start / COAST stop 打印
                time.sleep_ms(20)
            time.sleep_ms(1200)                    # 停顿, 确认已停稳
        print("---- [重捕获] 续追中途目标回到中心 → 应立即停 ----")
        gimbal_send(250, 0, 250, "track")
        time.sleep_ms(100)
        _c0 = time.ticks_ms()
        while _tdiff(time.ticks_ms(), _c0) < M_COAST_MS // 2:
            gimbal_send(0, 0, 0, "lost")
            time.sleep_ms(20)
        gimbal_send(0, 0, 0, "track")              # 目标回到画面中心(死区内) → 停转, 同时复位预判
        time.sleep_ms(100)
        _c0 = time.ticks_ms()
        while _tdiff(time.ticks_ms(), _c0) < 1000:
            gimbal_send(0, 0, 0, "lost")           # 继续丢靶: 若复位失效, 这里会暴露(继续转/打印 COAST stop)
            time.sleep_ms(20)
        print("  验证点: 上一步后云台应保持静止, 且无 COAST start/stop 打印")
        print(">>> COAST TEST done (Ctrl+C 退出)")
        while not app.need_exit():
            time.sleep_ms(1000)

# 电机测试模式优先执行(不跑视觉)
if MOTOR_ENABLED and mv.get("test_mode", False) and _yaw:
    import time as _t

    _rpm = mv.get("test_rpm", 10)
    _dirn = M_REV if M_REV else 1
    print(">>> MOTOR TEST (Emm42 步进, 持续旋转自检)")
    print("  串口: UART%d @%dbps, ID=%d, 校验=固定0x6B" % (M_PORT, M_BAUD, M_ADDR))
    print("  接线: MaixCAM2 TX=%s -> 驱动器RXD,  RX=%s -> 驱动器TXD" % (M_TX, M_RX))
    print("  共地 + 驱动器 V+ 独立供电 7~32V(勿接 MaixCAM2 5V)")
    _v = _yaw.read_bus_voltage()
    if _v:
        print("  在线探测: OK, 母线电压 %dmV" % _v)
    else:
        print("  在线探测: 无应答")

        def _hx(b):
            return " ".join("%02X" % x for x in b)

        _sent, _raw = _yaw.raw_probe()
        if not _raw:
            print("  RX 原始字节: (无) —— 一个字节都没收到")
            print("    → MaixCAM2 发出去了但没人回。依次查:")
            print("      1. 驱动器 OLED 是否正常 / 显示什么(Waiting V+ Power!?)")
            print("      2. 共地: MaixCAM2 GND 与驱动器电源 GND 是否连通")
            print("      3. 交叉接线: TX %s -> 驱动器RXD, RX %s -> 驱动器TXD" %
                  (M_TX, M_RX))
            print("      4. 驱动器菜单: 串口功能=UART_FUN(不能是 P_Pul/RxTx_OFF),")
            print("         串口ID=%d, 波特率=%d, 校验方式=固定0x6B, 应答=Receive" %
                  (M_ADDR, M_BAUD))
        elif _raw == _sent:
            print("  RX 原始字节: %s   <- 正是我们自己发出去的帧" % _hx(_raw))
            print("    → 回环测试通过: MaixCAM2 的 UART%d(%s/%s) 收发都正常" %
                  (M_PORT, M_TX, M_RX))
            print("      拔掉 TX/RX 之间的短接线, 接驱动器")
        else:
            print("  RX 原始字节: %s" % _hx(_raw))
            print("    (本机发出的是: %s)" % _hx(_sent))
            print("    → 收到字节但不是自身回环: 多半驱动器「校验方式」不是 固定0x6B")
    _yaw.enable(True)
    _t.sleep_ms(100)
    print("\n  持续%s %dRPM (reverse=%s; Ctrl+C 退出)" %
          ("正转" if _dirn > 0 else "反转", _rpm, mv.get("reverse", False)))
    print("  电机不动 → 先看上面'在线探测'是否 OK")
    while not app.need_exit():
        _yaw.set_speed(_dirn, _rpm)     # 持续重发, 防止丢帧后停住
        _t.sleep_ms(500)

# pitch 步进测试模式(不跑视觉): 在线探测 + 持续旋转自检(与 yaw 相同逻辑)
if PITCH_ENABLED and pm.get("test_mode", False) and _pitch:
    import time as _t

    _rpm = pm.get("test_rpm", 10)
    _dirn = P_REV if P_REV else 1
    print(">>> PITCH TEST (Emm42 步进, 持续旋转自检)")
    print("  串口: UART%d @%dbps, ID=%d, 校验=固定0x6B" % (P_PORT, P_BAUD, P_ADDR))
    print("  接线: MaixCAM2 TX=%s -> 驱动器RXD,  RX=%s -> 驱动器TXD" % (P_TX, P_RX))
    print("  共地 + 驱动器 V+ 独立供电 7~32V(勿接 MaixCAM2 5V)")
    _v = _pitch.read_bus_voltage()
    if _v:
        print("  在线探测: OK, 母线电压 %dmV" % _v)
    else:
        print("  在线探测: 无应答")

        def _hx(b):
            return " ".join("%02X" % x for x in b)

        _sent, _raw = _pitch.raw_probe()
        if not _raw:
            print("  RX 原始字节: (无) —— 一个字节都没收到")
            print("    → MaixCAM2 发出去了但没人回。依次查:")
            print("      1. 驱动器 OLED 是否正常 / 显示什么(Waiting V+ Power!?)")
            print("      2. 共地: MaixCAM2 GND 与驱动器电源 GND 是否连通")
            print("      3. 交叉接线: TX %s -> 驱动器RXD, RX %s -> 驱动器TXD" % (P_TX, P_RX))
            print("      4. 驱动器菜单: 串口功能=UART_FUN, 串口ID=%d, 波特率=%d," % (P_ADDR, P_BAUD))
            print("         校验方式=固定0x6B, 应答=Receive")
        elif _raw == _sent:
            print("  RX 原始字节: %s   <- 正是我们自己发出去的帧" % _hx(_raw))
            print("    → 回环测试通过: MaixCAM2 的 UART%d(%s/%s) 收发都正常" % (P_PORT, P_TX, P_RX))
            print("      拔掉 TX/RX 之间的短接线, 接驱动器")
        else:
            print("  RX 原始字节: %s" % _hx(_raw))
            print("    (本机发出的是: %s)" % _hx(_sent))
            print("    → 收到字节但不是自身回环: 多半驱动器「校验方式」不是 固定0x6B")
    _pitch.enable(True)
    _t.sleep_ms(100)
    print("\n  持续%s %dRPM (reverse=%s; Ctrl+C 退出)" %
          ("正转" if _dirn > 0 else "反转", _rpm, pm.get("reverse", False)))
    print("  电机不动 → 先看上面'在线探测'是否 OK")
    while not app.need_exit():
        _pitch.set_speed(_dirn, _rpm)     # 持续重发, 防止丢帧后停住
        _t.sleep_ms(500)

if SERVO_ENABLED and S_TEST_MODE:
    _servo_self_test()

print("boot: 进入主循环 (mode=%s)" % MODE)
if MODE == "yolo":
    if _yolo_ok():
        run_yolo()
    else:
        print("YOLO libs missing, fallback to CV")
        run_cv()
else:
    run_cv()
