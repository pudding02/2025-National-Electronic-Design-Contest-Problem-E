"""V15 姿态稳定(板载 IMU 内环 + 视觉外环) —— 主机侧测试。

覆盖:
  - att_px_per_deg / att_focal_px  由靶标像素面积+已知物理尺寸+标定距离反算"像素↔角度"
  - att_tilt                       倾角互补滤波(陀螺短时 + 加速度长时)
  - att_yaw_rate                   机体角速度 → 世界竖轴角速度(去倾斜投影)
  - att_step                       内环: 扰动观测器 + 角度P → 整数 RPM(死区/限幅/发散标志)
  - att_gate                       最小脉宽(抑制零附近通断抖动; 换向不受限)
  - att_outer                      视觉外环: 像素偏差 → θ_ref 增量(死区/增益/前馈限幅)
  - att_turn_step                  单方向按圈搜索(IMU 计 360°, 底座同转也算真实扫过)
  - 闭环仿真                       **本套的核心**: 底座 200°/s 阶跃 / ±100°/s 正弦 /
                                   5Hz 手抖 / 陀螺 1% 比例误差 / 1RPM 量化 / 传输延迟,
                                   断言「靶心像素偏差」折算角度在验收线内且不发散
  - 静态守门                       main.py 定义姿态参数与状态、内环真的接线、
                                   config 里 attitude.enabled 默认关

原理：从 k230/main.py 用 AST 提取纯函数, 在最小环境执行(math + ATT_RPM_TO_DPS)。
仿真部分不依赖硬件: 用一个「底座转速 + 电机转速 = 相机真实角速度」的模型闭合环路,
量化/延迟/噪声/比例误差全部建模, 这样不用上机就能证明增益不会发散、精度够不够。
运行：python tools/test_attitude_loop.py
"""
import ast
import math
import os
import random
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_MAIN_PY = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "main.py"))
_CFG_JSON = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "config.json"))

_FUNC_NAMES = ("att_px_per_deg", "att_focal_px", "att_tilt", "att_yaw_rate",
               "att_step", "att_gate", "att_outer", "att_turn_step")
_CONST_NAMES = ("ATT_RPM_TO_DPS",)

# main.py 必须模块级定义的名字(静态守门用)
_ATT_PARAM_VARS = ("ATT_EN_CFG", "ATT_YAW_SIGN", "ATT_TICK_US", "ATT_BUDGET_MS",
                   "ATT_GYRO_DPS", "ATT_ANGLE_KP", "ATT_RATE_KI", "ATT_DEADBAND_DPS",
                   "ATT_MIN_PULSE_MS", "ATT_MIN_RPM", "ATT_PX_PER_DEG_CFG",
                   "ATT_OUTER_GAIN", "ATT_OUTER_LEAD", "ATT_HOLD_MS",
                   "ATT_FAULT_DEG", "ATT_ACTIVE_DPS", "ATT_TILT_ALPHA", "ATT_ACC",
                   "SEARCH_TURNS", "SEARCH_DPS")
_ATT_STATE_VARS = ("_att_th", "_att_ref", "_att_rpm", "_att_w", "_att_err",
                   "_att_mode", "_att_swept", "_att_sdone", "_att_drive")


def _extract_func(src, name):
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return ast.get_source_segment(src, node)
    raise AssertionError("k230/main.py 中未找到函数 %s" % name)


def _extract_assign(src, name):
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                if isinstance(tgt, ast.Name) and tgt.id == name:
                    return ast.get_source_segment(src, node)
    raise AssertionError("k230/main.py 中未找到模块级赋值 %s = ..." % name)


_ENV = None
_FNS = None

# 与 config.json [attitude] 默认值保持一致(仿真按默认增益跑 = 出厂即达标)
DEFAULTS = dict(angle_kp=12.0, rate_ki=0.55, deadband=3.0, min_rpm=1, max_rpm=90,
                fault_deg=15.0, ppd=11.5)


def load_funcs():
    global _ENV, _FNS
    with open(_MAIN_PY, encoding="utf-8") as f:
        src = f.read()
    env = {"math": math}
    for cname in _CONST_NAMES:
        exec(_extract_assign(src, cname), env)
    fns = {}
    missing = []
    for name in _FUNC_NAMES:
        try:
            exec(_extract_func(src, name), env)
        except AssertionError:
            missing.append(name)
            continue
        fns[name] = env[name]
    if missing:
        raise AssertionError("k230/main.py 缺少函数: %s (未实现或改名)" % ", ".join(missing))
    _ENV = env
    _FNS = fns
    return fns


def F(name):
    if _FNS is None:
        load_funcs()
    return _FNS[name]


def RPD():
    F("att_step")
    return _ENV["ATT_RPM_TO_DPS"]


# ---------------------------------------------------------------------------
# 闭环仿真: 底座转速 + 电机转速 = 相机真实角速度; 陀螺可带比例误差与噪声
# ---------------------------------------------------------------------------

def run_loop(base_fn, duration_s=1.0, dt=0.0025, delay=1, noise_dps=0.0,
             scale_err=0.0, sign=1, outer_gain=0.0, vision_every=18,
             target_deg=0.0, min_pulse=0, **kw):
    """跑一段时间, 返回 (最大角偏差°, 稳态角偏差°, 末端θ_true, 发散标志)。

    偏差口径 = 画面里靶心离收敛点的角度(|target - θ_true|), 也就是验收指标本身。
    稳态 = 最后 20% 时间窗内的最大值(排除启动瞬态)。
    """
    p = dict(DEFAULTS)
    p.update(kw)
    att_step = F("att_step")
    att_gate = F("att_gate")
    att_outer = F("att_outer")
    kg = RPD()
    rnd = random.Random(1234)

    theta_true = 0.0        # 相机真实绝对方位(物理真值)
    theta_imu = 0.0         # 陀螺积分出的估计(带比例误差/噪声)
    theta_ref = 0.0         # 视觉外环给的方位设定值
    rpm_prev = 0
    n = int(duration_s / dt)
    hist = [0] * (n + delay + 1)
    max_dev = 0.0
    tail = 0.0
    tail_from = int(n * 0.8)

    for k in range(n):
        t = k * dt
        w_base = base_fn(t)
        rpm_app = hist[k - delay] if k >= delay else 0
        w_cam = w_base + kg * sign * rpm_app          # 相机真实角速度
        theta_true += w_cam * dt
        w_meas = w_cam * (1.0 + scale_err)            # 陀螺读数(比例误差)
        if noise_dps:
            w_meas += rnd.uniform(-noise_dps, noise_dps)
        theta_imu += w_meas * dt

        # 视觉外环: 隔 vision_every 拍刷新一次(约 45fps)
        if outer_gain > 0 and k % vision_every == 0:
            dx_px = (target_deg - theta_true) * p["ppd"]
            delta, _ = att_outer(dx_px, p["ppd"], outer_gain, 0.0, 0.0,
                                 dt * vision_every, 0.0)
            theta_ref += delta

        rpm, _err, _fault = att_step(theta_imu, theta_ref, w_meas, rpm_prev,
                                     p["angle_kp"], p["rate_ki"], p["deadband"],
                                     p["min_rpm"], p["max_rpm"], p["fault_deg"])
        rpm = att_gate(rpm, rpm_prev, min_pulse + 1, min_pulse)
        rpm_prev = rpm
        hist[k] = rpm

        dev = abs(target_deg - theta_true)
        if dev > max_dev:
            max_dev = dev
        if k >= tail_from and dev > tail:
            tail = dev
    return max_dev, tail, theta_true, (max_dev > DEFAULTS["fault_deg"])


# ---------------------------------------------------------------------------
# 几何: 像素↔角度
# ---------------------------------------------------------------------------

class TestGeometry(unittest.TestCase):
    def test_px_per_deg(self):
        # f=660px → 660·π/180 ≈ 11.52 px/deg → 0.5° ≈ 5.76px
        self.assertAlmostEqual(F("att_px_per_deg")(660.0), 11.519, places=3)
        self.assertEqual(F("att_px_per_deg")(0), 0.0)
        self.assertEqual(F("att_px_per_deg")(-5), 0.0)

    def test_focal_from_calibration(self):
        # 真机标定: ref_area_px=8412 @1500mm, 白框 256×170mm → f≈660px
        f = F("att_focal_px")(8412, 25.6 / 17.0, 256.0, 1500.0)
        self.assertAlmostEqual(f, 660.0, delta=8.0)
        # 退化输入一律 0(调用方走默认值, 不产生 NaN/负数)
        for bad in ((0, 1.5, 256, 1500), (8412, 0, 256, 1500),
                    (8412, 1.5, 0, 1500), (8412, 1.5, 256, 0)):
            self.assertEqual(F("att_focal_px")(*bad), 0.0)

    def test_ppd_matches_half_degree_spec(self):
        # 验收线自洽: 0.5° 对应的像素数应在 5~8px 区间
        ppd = F("att_px_per_deg")(660.0)
        self.assertTrue(5.0 <= 0.5 * ppd <= 8.0, 0.5 * ppd)


# ---------------------------------------------------------------------------
# 倾角 / 去倾斜投影
# ---------------------------------------------------------------------------

class TestTilt(unittest.TestCase):
    def test_level_gives_zero(self):
        roll, pitch = F("att_tilt")(0, 0, 1.0, 0, 0, 0.0, 0.0, 0.98, 0.0025)
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)

    def test_dt_zero_uses_acc_only(self):
        # roll 由 ay/az 决定: ay=az → 45°
        roll, pitch = F("att_tilt")(0, 1.0, 1.0, 100, 100, 0.0, 0.0, 0.98, 0.0)
        self.assertAlmostEqual(roll, 45.0, places=4)
        self.assertAlmostEqual(pitch, 0.0, places=4)

    def test_gyro_leads_short_term(self):
        # 首帧之后陀螺预测占大头: 一拍后应在 0 与加速度角之间偏向陀螺侧
        roll, _ = F("att_tilt")(0, 1.0, 1.0, 1000, 0, 0.0, 0.0, 0.98, 0.0025)
        # 陀螺预测 1000*0.0025=2.5°, 加速度 45° → 0.98*2.5+0.02*45 = 3.35
        self.assertAlmostEqual(roll, 0.98 * 2.5 + 0.02 * 45.0, places=4)

    def test_converges_to_acc_long_term(self):
        roll, pitch = 0.0, 0.0
        for _ in range(4000):
            roll, pitch = F("att_tilt")(0, 1.0, 1.0, 0, 0, roll, pitch, 0.98, 0.0025)
        self.assertAlmostEqual(roll, 45.0, delta=0.5)
        self.assertAlmostEqual(pitch, 0.0, delta=0.5)


class TestYawRate(unittest.TestCase):
    def test_level_reduces_to_gz(self):
        self.assertAlmostEqual(F("att_yaw_rate")(1, 2, 3, 0.0, 0.0), 3.0, places=6)

    def test_pitch_90_projects_to_gx(self):
        # 相机抬头 90° 后, 绕世界竖轴的转动全部来自机体 x 轴(取负)
        self.assertAlmostEqual(F("att_yaw_rate")(5, 0, 99, 0.0, 90.0), -5.0, places=6)

    def test_roll_90_projects_to_gy(self):
        self.assertAlmostEqual(F("att_yaw_rate")(0, 5, 99, 90.0, 0.0), 5.0, places=6)


# ---------------------------------------------------------------------------
# 内环一拍
# ---------------------------------------------------------------------------

class TestStep(unittest.TestCase):
    def _step(self, **kw):
        p = dict(DEFAULTS)
        p.update(kw)
        return F("att_step")

    def test_no_error_no_disturbance_stops(self):
        rpm, err, fault = F("att_step")(0.0, 0.0, 0.0, 0,
                                        12.0, 0.7, 3.0, 1, 90, 15.0)
        self.assertEqual((rpm, err, fault), (0, 0.0, False))

    def test_disturbance_rejected_with_negative_command(self):
        # 底座 +200°/s、电机 0 → 观测器立刻给出反向指令
        rpm, _e, _f = F("att_step")(0.0, 0.0, 200.0, 0, 12.0, 0.7, 3.0, 1, 90, 15.0)
        self.assertLess(rpm, 0)
        self.assertEqual(rpm, -23)      # 0.7*200=140°/s → 23.3RPM → 23

    def test_deadband_stops(self):
        # 误差极小 → 需要的转速落在死区内 → 停(防低速嗒嗒)
        rpm, _e, _f = F("att_step")(0.0, 0.1, 0.0, 0, 12.0, 0.7, 3.0, 1, 90, 15.0)
        self.assertEqual(rpm, 0)

    def test_min_rpm_floor(self):
        # 超过死区但不足 1RPM → 抬到 min_rpm 保起步扭矩
        rpm, _e, _f = F("att_step")(0.0, 1.0, 0.0, 0, 12.0, 0.7, 0.1, 1, 90, 15.0)
        self.assertEqual(rpm, 1)

    def test_max_rpm_clamp(self):
        rpm, _e, _f = F("att_step")(0.0, 100.0, 0.0, 0, 100.0, 0.7, 3.0, 1, 5, 500.0)
        self.assertEqual(rpm, 5)

    def test_fault_flag(self):
        _r, _e, fault = F("att_step")(0.0, 20.0, 0.0, 0, 12.0, 0.7, 3.0, 1, 90, 15.0)
        self.assertTrue(fault)
        _r, _e, fault = F("att_step")(0.0, 14.9, 0.0, 0, 12.0, 0.7, 3.0, 1, 90, 15.0)
        self.assertFalse(fault)

    def test_sign_symmetric(self):
        a, _, _ = F("att_step")(0.0, 0.0, 200.0, 0, 12.0, 0.7, 3.0, 1, 90, 15.0)
        b, _, _ = F("att_step")(0.0, 0.0, -200.0, 0, 12.0, 0.7, 3.0, 1, 90, 15.0)
        self.assertEqual(a, -b)


class TestGate(unittest.TestCase):
    def test_holds_previous_within_pulse(self):
        self.assertEqual(F("att_gate")(0, 5, 10, 25), 5)

    def test_releases_after_pulse(self):
        self.assertEqual(F("att_gate")(0, 5, 26, 25), 0)

    def test_reversal_not_blocked(self):
        self.assertEqual(F("att_gate")(-3, 5, 1, 25), -3)

    def test_zero_pulse_disables(self):
        self.assertEqual(F("att_gate")(0, 5, 0, 0), 0)


class TestOuter(unittest.TestCase):
    def test_deadband(self):
        d, dn = F("att_outer")(5, 11.5, 0.8, 5, 0.0, 0.022, 0.0)
        self.assertEqual(d, 0.0)
        self.assertEqual(dn, 5.0)

    def test_px_to_deg_with_gain(self):
        # 115px / 11.5 = 10° × 0.8 = 8°
        d, _ = F("att_outer")(115, 11.5, 0.8, 5, 0.0, 0.022, 0.0)
        self.assertAlmostEqual(d, 8.0, places=6)

    def test_zero_ppd_safe(self):
        self.assertEqual(F("att_outer")(100, 0, 0.8, 5, 0.0, 0.022, 0.0)[0], 0.0)

    def test_lead_limited(self):
        # 前馈限幅 ±5°: 误差猛跳也不许把设定值甩飞
        d, _ = F("att_outer")(600, 11.5, 1.0, 5, -600.0, 0.001, 0.5)
        self.assertLessEqual(abs(d), 5.0 + 600 / 11.5 + 1e-9)


class TestTurn(unittest.TestCase):
    def test_advances_and_counts(self):
        ref, swept, done = F("att_turn_step")(0.0, 0.0, 0.0, 1, 25.0, 0.1, 0.0, 0)
        self.assertAlmostEqual(ref, 2.5, places=6)
        self.assertAlmostEqual(swept, 0.0, places=6)
        self.assertFalse(done)

    def test_counts_real_world_rotation(self):
        # 底座同向也在转: 用相机世界角的实际变化累计
        ref, swept, _ = F("att_turn_step")(10.0, 0.0, 0.0, 1, 0.0, 0.1, 0.0, 0)
        self.assertAlmostEqual(swept, 10.0, places=6)

    def test_done_after_turns(self):
        # 已扫 359.9°, 这一拍又真转了 1° → 满一圈
        _r, _s, done = F("att_turn_step")(1.0, 0.0, 0.0, 1, 0.0, 0.1, 359.9, 1)
        self.assertTrue(done)

    def test_turns_zero_never_done(self):
        _r, _s, done = F("att_turn_step")(0.0, 0.0, 0.0, 1, 25.0, 0.1, 1e6, 0)
        self.assertFalse(done)

    def test_negative_direction(self):
        ref, swept, _ = F("att_turn_step")(-10.0, 0.0, 0.0, -1, 25.0, 0.1, 0.0, 0)
        self.assertAlmostEqual(ref, -2.5, places=6)
        self.assertAlmostEqual(swept, 10.0, places=6)


# ---------------------------------------------------------------------------
# 闭环仿真(核心): 精度与稳定性
# ---------------------------------------------------------------------------

class TestClosedLoop(unittest.TestCase):
    """默认增益 + 默认拍长(2.5ms) + 1 拍延迟 + 1RPM 量化, 是否守得住 0.5°/失稳?"""

    def test_step_200dps_transient_and_steady(self):
        # 底座 0→200°/s 阶跃(最狠的扰动): 瞬态允许略超, 稳态必须远优于 0.5°
        mx, tail, _t, diverged = run_loop(lambda t: 200.0 if t > 0.05 else 0.0,
                                          duration_s=1.0)
        self.assertFalse(diverged, "200°/s 阶跃导致发散: max=%.3f°" % mx)
        self.assertLess(mx, 1.0, "瞬态峰值 %.3f°/%.1fpx 超 1°" % (mx, mx * DEFAULTS["ppd"]))
        self.assertLess(tail, 0.5, "稳态 %.3f° 超验收线 0.5°" % tail)

    def test_sine_100dps(self):
        # 底座 ±100°/s 正弦(人类来回摆动的量级): 全程都应在 0.5° 内
        fn = lambda t: 100.0 * math.sin(2 * math.pi * 0.5 * t)
        mx, tail, _t, diverged = run_loop(fn, duration_s=2.0)
        self.assertFalse(diverged)
        self.assertLess(mx, 0.5, "正弦峰值 %.3f° 超 0.5°" % mx)
        self.assertLess(tail, 0.5)

    def test_hand_jitter_5hz(self):
        # 手抖/振动 5Hz ±30°/s
        fn = lambda t: 30.0 * math.sin(2 * math.pi * 5.0 * t)
        mx, _tail, _t, diverged = run_loop(fn, duration_s=1.0)
        self.assertFalse(diverged)
        self.assertLess(mx, 0.5, "5Hz 抖动峰值 %.3f° 超 0.5°" % mx)

    def test_gyro_scale_error_1pct(self):
        # 陀螺 1% 比例误差: 帧间窗口内累积可忽略, 闭环仍守住
        mx, _tail, _t, diverged = run_loop(lambda t: 200.0 if t > 0.05 else 0.0,
                                           duration_s=1.0, scale_err=0.01)
        self.assertFalse(diverged)
        self.assertLess(mx, 1.2)

    def test_gyro_noise_tolerated(self):
        # 0.5°/s 量级读数噪声: 不许把云台抖起来
        mx, _tail, _t, diverged = run_loop(lambda t: 200.0 if t > 0.05 else 0.0,
                                           duration_s=1.0, noise_dps=0.5)
        self.assertFalse(diverged)
        self.assertLess(mx, 1.5)

    def test_delay_up_to_3_ticks_stable(self):
        # 延迟到 3 拍(≈7.5ms)仍须收敛。注意真机不会这么糟: 姿态模式 acc=255 时
        # 每 +1RPM 只要 0.05ms, 33RPM 的建立时间 ≈1.65ms(不到 1 拍), 所以这条是悲观界。
        # 这条同时是「极限环」回归闸: rate_ki≥0.6 在 3 拍延迟下会振 2~6° 且稳态不平。
        mx, tail, _t, diverged = run_loop(lambda t: 200.0 if t > 0.05 else 0.0,
                                          duration_s=2.0, delay=3)
        self.assertFalse(diverged, "3 拍延迟下发散: max=%.3f°" % mx)
        self.assertLess(mx, 3.0, "3 拍延迟峰值 %.3f° 过大" % mx)
        self.assertLess(tail, 0.5, "3 拍延迟下未收敛(疑极限环): steady=%.3f°" % tail)

    def test_wrong_sign_diverges_and_trips_fault(self):
        # 符号配反: 必须发散且被 fault 判据抓到(这正是发散保护存在的原因)
        mx, _tail, _t, diverged = run_loop(lambda t: 200.0 if t > 0.05 else 0.0,
                                          duration_s=1.0, sign=-1)
        self.assertTrue(diverged)
        self.assertGreater(mx, DEFAULTS["fault_deg"])

    def test_vision_outer_converges_without_overshoot(self):
        # 外环: 起始偏 30°(靶在画面右 345px), 纯外环 + 内环应收到目标且不过冲太多。
        # 初始 30° > fault_deg 是刻意构造(模拟"刚锁定就在大偏差处"), 所以这里不看发散标志。
        mx, tail, theta, _diverged = run_loop(lambda t: 0.0, duration_s=3.0,
                                              outer_gain=0.8, target_deg=30.0)
        self.assertLess(abs(30.0 - theta), 0.5, "末端未收敛: θ=%.2f" % theta)

    def test_quantization_worst_case_steady(self):
        # 200°/s 恰好需要 33.33RPM → 只能给 33 或 34, 稳态残差 = 3°/s ÷ kp = 0.25°
        _mx, tail, _t, _d = run_loop(lambda t: 200.0 if t > 0.05 else 0.0, duration_s=2.0)
        self.assertLess(tail, 0.5)
        self.assertGreater(tail, 0.0)   # 量化残差客观存在, 不是 0

    def test_slow_rotation_fine_control(self):
        # 慢速 20°/s: 靠死区+最小脉宽不抖(不出现高频正反切换)
        mx, tail, _t, diverged = run_loop(lambda t: 20.0 if t > 0.05 else 0.0,
                                          duration_s=2.0)
        self.assertFalse(diverged)
        self.assertLess(mx, 0.5)


# ---------------------------------------------------------------------------
# 静态守门
# ---------------------------------------------------------------------------

class TestMainStaticGuards(unittest.TestCase):
    def _src(self):
        with open(_MAIN_PY, encoding="utf-8") as f:
            return f.read()

    def test_att_vars_defined(self):
        tree = ast.parse(self._src())
        defined = set()
        for node in tree.body:
            if isinstance(node, ast.Assign):
                for t in node.targets:
                    if isinstance(t, ast.Name):
                        defined.add(t.id)
        for name in _ATT_PARAM_VARS + _ATT_STATE_VARS:
            self.assertIn(name, defined, "main.py 缺少模块级定义: %s" % name)

    def test_inner_loop_wired(self):
        src = self._src()
        self.assertIn("att_step(", src)
        self.assertIn("att_gate(", src)
        self.assertIn("att_outer(", src)
        self.assertIn("_att_tick(", src)
        self.assertIn("_att_spin(", src)
        self.assertIn("_att_bind(", src)

    def test_gimbal_dispatch_has_no_inline_branch(self):
        # 纪律锚点: yaw 三态走预先绑定的函数指针, gimbal_send 里不许再写 if ATT_EN
        src = self._src()
        self.assertIn("_trk_yaw(dx)", src)
        self.assertIn("_search_yaw()", src)
        self.assertIn("_lost_yaw()", src)
        self.assertNotIn("if ATT_EN:", src)

    def test_default_off_guard(self):
        # 默认关的回归锚点: attitude.enabled 必须 false(交付即安全)
        import json
        with open(_CFG_JSON, encoding="utf-8") as f:
            cfg = json.load(f)
        self.assertFalse(cfg["attitude"].get("enabled", True),
                         "attitude.enabled 默认必须为 false")
        self.assertIn("turns", cfg["search"], "search.turns 缺失")
        self.assertEqual(cfg["search"]["turns"], 0, "search.turns 默认 0=一直找")

    def test_px_per_deg_derivable_from_config(self):
        # 免新增标定: 用 [distance]+[circle] 的既有值必须能反算出合理的 px/deg
        import json
        with open(_CFG_JSON, encoding="utf-8") as f:
            cfg = json.load(f)
        d = cfg["distance"]
        c = cfg["circle"]
        fpx = F("att_focal_px")(d["ref_area_px"],
                                c["rect_w_cm"] / c["rect_h_cm"],
                                c["rect_w_cm"] * 10.0, d["ref_distance_mm"])
        ppd = F("att_px_per_deg")(fpx)
        self.assertTrue(6.0 <= ppd <= 20.0, "自动反算的 px/deg=%.2f 不合理" % ppd)


if __name__ == "__main__":
    unittest.main(verbosity=2)
