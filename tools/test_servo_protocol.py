"""FashionStar FSUS 双舵机协议/控制纯函数主机侧测试。

原理: 用 AST 从 k230/main.py 提取 `fs_*` 帧构造/解析函数与 `servo_*` 控制纯函数,
在主机 Python 里执行, 与协议帧表/串口实测样本逐字节对拍; 不需要 MaixCAM2 硬件。

运行:  python tools/test_servo_protocol.py
"""
import ast
import json
import os
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_MAIN_PY = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "main.py"))
_CFG = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "config.json"))

_FS_NAMES = {
    "FS_ANGLE_MIN", "FS_ANGLE_MAX",
    "fs_u16", "fs_u32", "fs_i32", "fs_i32_at", "fs_i16_at",
    "fs_frame", "fs_ping", "fs_set_mturn", "fs_set_single",
    "fs_sync_mturn", "fs_stop", "fs_query_mturn", "fs_query_single",
    "fs_parse", "fs_angle_mturn", "fs_angle_single",
    "servo_pid", "servo_step_target", "servo_search_step", "servo_boost",
    "_servo_send_targets", "_servo_boot_send", "servo_send",
}


def load_env():
    with open(_MAIN_PY, encoding="utf-8") as f:
        src = f.read()
    tree = ast.parse(src)
    env = {}
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = {t.id for t in node.targets if isinstance(t, ast.Name)}
            if names & {"FS_ANGLE_MIN", "FS_ANGLE_MAX"}:
                exec(ast.get_source_segment(src, node), env)
        elif isinstance(node, ast.FunctionDef) and node.name in _FS_NAMES:
            exec(ast.get_source_segment(src, node), env)
    missing = _FS_NAMES - set(env)
    assert not missing, "k230/main.py 缺函数/常量: %s" % sorted(missing)
    return env


E = load_env()


def hx(s):
    return bytes.fromhex(s)


class TestFrames(unittest.TestCase):
    def test_ping_id0(self):
        self.assertEqual(E["fs_ping"](0), hx("12 4C 01 01 00 60"))

    def test_ping_id1(self):
        self.assertEqual(E["fs_ping"](1), hx("12 4C 01 01 01 61"))

    def test_set_mturn_id0_1800deg_5000ms(self):
        # 协议帧表实测: +1800°(5圈), 5000ms
        self.assertEqual(E["fs_set_mturn"](0, 1800.0, 5000, 0),
                         hx("12 4C 0D 0B 00 50 46 00 00 88 13 00 00 00 00 A7"))

    def test_set_mturn_id1_minus360deg_3000ms(self):
        # 串口调试.md 实测: ID=1 反转 -360°, 3000ms
        self.assertEqual(E["fs_set_mturn"](1, -360.0, 3000, 0),
                         hx("12 4C 0D 0B 01 F0 F1 FF FF B8 0B 00 00 00 00 19"))

    def test_set_single_id0_90deg_1500ms(self):
        self.assertEqual(E["fs_set_single"](0, 90.0, 1500, 0),
                         hx("12 4C 08 07 00 84 03 DC 05 00 00 D5"))

    def test_stop_id0_unload(self):
        self.assertEqual(E["fs_stop"](0, 0, 0),
                         hx("12 4C 18 04 00 10 00 00 8A"))

    def test_stop_id0_hold(self):
        self.assertEqual(E["fs_stop"](0, 1, 0),
                         hx("12 4C 18 04 00 11 00 00 8B"))

    def test_sync_mturn_two_axes(self):
        f = E["fs_sync_mturn"]([(0, 90.0), (1, -90.0)], 40, 0)
        # 12 4C 19 19 [0D 0B 02] (00 84 03 00 00 28 00 00 00 00 00)
        #                    (01 7C FC FF FF 28 00 00 00 00 00) cs
        want = bytearray(hx(
            "12 4C 19 19 0D 0B 02 "
            "00 84 03 00 00 28 00 00 00 00 00 "
            "01 7C FC FF FF 28 00 00 00 00 00"))
        want.append(sum(want) & 0xFF)
        self.assertEqual(f, bytes(want))

    def test_sync_mturn_len_and_checksum(self):
        f = E["fs_sync_mturn"]([(0, 0.0), (1, 0.0)], 35, 0)
        self.assertEqual(len(f), 30)
        self.assertEqual(f[0:2], hx("12 4C"))
        self.assertEqual(f[2], 0x19)
        self.assertEqual(f[3], 25)
        self.assertEqual(f[-1], sum(f[:-1]) & 0xFF)

    def test_parse_ok(self):
        # cmd16 响应实测: 05 1C 10 07 00 50 2F 00 00 03 00 BA
        b = hx("05 1C 10 07 00 50 2F 00 00 03 00 BA")
        r = E["fs_parse"](b)
        self.assertIsNotNone(r)
        self.assertEqual(r[0], 0x10)
        self.assertEqual(len(r[1]), 7)
        self.assertAlmostEqual(E["fs_angle_mturn"](r[1]), 1211.2)

    def test_parse_bad_checksum(self):
        b = bytearray(hx("05 1C 10 07 00 50 2F 00 00 03 00 BA"))
        b[-1] ^= 0xFF
        self.assertIsNone(E["fs_parse"](bytes(b)))

    def test_parse_short(self):
        self.assertIsNone(E["fs_parse"](b"\x05\x1c\x01"))

    def test_angle_single(self):
        # cmd10 响应: 05 1C 0A 03 00 84 03 B5 -> 900 -> 90.0°
        b = hx("05 1C 0A 03 00 84 03 B5")
        r = E["fs_parse"](b)
        self.assertIsNotNone(r)
        self.assertAlmostEqual(E["fs_angle_single"](r[1]), 90.0)

    def test_negative_i32_twos_complement(self):
        f = E["fs_set_mturn"](0, -90.0, 0, 0)
        # content: id + 7C FC FF FF + 00 00 00 00 + 00 00
        self.assertEqual(f[4:10], hx("00 7C FC FF FF 00"))

    def test_query_data_frame_matches_doc(self):
        # 文档 §20.2 示例: 读 ID0 的 data_id 0x03(帧构造保留为纯函数, 运行期不用)
        self.assertEqual(E["fs_frame"](0x03, hx("00 03")), hx("12 4C 03 02 00 03 66"))


class TestServoControl(unittest.TestCase):
    def test_deadband_zero_output(self):
        u, i, _ = E["servo_pid"](2.0, 0.0, 5.0, 0.033, 0.2, 0.1, 0.0, 0.0, None,
                                 150.0, 3.0, 60.0)
        self.assertEqual(u, 0.0)
        self.assertEqual(i, 0.0)

    def test_p_only(self):
        u, i, _ = E["servo_pid"](100.0, 0.0, 0.0, 0.033, 0.2, 0.0, 0.0, 0.0, None,
                                 1000.0, 0.0, 60.0)
        self.assertAlmostEqual(u, 20.0)
        self.assertAlmostEqual(i, 0.0)

    def test_output_clamp_and_antiwindup(self):
        # kp=1, ki=1, e=100, dt=0.033 -> P=100, I=3.3; vmax=50 -> clamp 50,
        # 误差为正且饱和 -> 冻结积分(保持 i_acc)
        u, i, _ = E["servo_pid"](100.0, 0.0, 7.0, 0.033, 1.0, 1.0, 0.0, 0.0, None,
                                 50.0, 0.0, 1000.0)
        self.assertAlmostEqual(u, 50.0)
        self.assertAlmostEqual(i, 7.0)

    def test_integral_separation(self):
        # |e|>=izone -> 不积分
        u, i, _ = E["servo_pid"](100.0, 0.0, 7.0, 0.033, 0.0, 1.0, 0.0, 0.0, None,
                                 1000.0, 0.0, 60.0)
        self.assertAlmostEqual(i, 7.0)

    def test_integral_accumulates(self):
        u, i, _ = E["servo_pid"](10.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, None,
                                 1000.0, 0.0, 60.0)
        self.assertAlmostEqual(i, 1.0)
        self.assertAlmostEqual(u, 1.0)

    def test_boost_saturates_outside_quarter_screen(self):
        # 640 宽 -> 1/4 = 160px; 超过就满速, 内侧仍按 PID
        self.assertEqual(E["servo_boost"](5.0, 200.0, 160.0, 260.0), 260.0)
        self.assertEqual(E["servo_boost"](5.0, -200.0, 160.0, 260.0), -260.0)
        self.assertEqual(E["servo_boost"](5.0, 160.0, 160.0, 260.0), 5.0)   # 边界不触发
        self.assertEqual(E["servo_boost"](5.0, 100.0, 160.0, 260.0), 5.0)

    def test_boost_disabled(self):
        self.assertEqual(E["servo_boost"](5.0, 500.0, 0.0, 260.0), 5.0)

    def test_derivative_and_feedforward(self):
        # e_vel=10, kd=0.1 -> +1; kff=0.02 -> +0.2; kp=0.1*10=1 -> 2.2
        u, _, _ = E["servo_pid"](10.0, 0.0, 0.0, 0.1, 0.1, 0.0, 0.1, 0.02, 10.0,
                                 1000.0, 0.0, 60.0)
        self.assertAlmostEqual(u, 2.2)

    def test_step_limit(self):
        nt, sat = E["servo_step_target"](0.0, 100.0, 0.033, -100.0, 100.0, 1.0)
        self.assertAlmostEqual(nt, 1.0)
        self.assertTrue(sat)

    def test_step_clamp_hi(self):
        nt, sat = E["servo_step_target"](9.5, 100.0, 0.1, -100.0, 10.0, 10.0)
        self.assertAlmostEqual(nt, 10.0)
        self.assertTrue(sat)

    def test_step_clamp_lo(self):
        nt, sat = E["servo_step_target"](-9.5, -100.0, 0.1, -10.0, 100.0, 10.0)
        self.assertAlmostEqual(nt, -10.0)
        self.assertTrue(sat)

    def test_search_forward(self):
        nt, d = E["servo_search_step"](0.0, 1, 30.0, 0.1, -10.0, 10.0)
        self.assertAlmostEqual(nt, 3.0)
        self.assertEqual(d, 1)

    def test_search_reverse_at_hi(self):
        nt, d = E["servo_search_step"](9.0, 1, 30.0, 0.1, -10.0, 10.0)
        self.assertAlmostEqual(nt, 10.0)
        self.assertEqual(d, -1)

    def test_search_no_jump_when_outside_band(self):
        # 追踪到 140° 后丢靶, 搜索带 ±120: 先以 3°/帧 回带内, 不瞬跳
        nt, d = E["servo_search_step"](140.0, 1, 30.0, 0.1, -120.0, 120.0)
        self.assertAlmostEqual(nt, 137.0)
        self.assertEqual(d, -1)


class TestStaticGuards(unittest.TestCase):
    def setUp(self):
        with open(_MAIN_PY, encoding="utf-8") as f:
            self.src = f.read()
        with open(_CFG, encoding="utf-8") as f:
            self.cfg = json.load(f)

    def test_servo_path_defined(self):
        for s in ("class FashionStarBus", "class FashionStarServo",
                  "def servo_send", "def _servo_init", "gimbal_send = servo_send",
                  "fs_sync_mturn", "SERVO_ENABLED"):
            self.assertIn(s, self.src, s)
        # 平台必须是 MaixCAM2 / MaixPy v4: uart.UART("/dev/ttySx") + pinmap
        for good in ('uart.UART("/dev/ttyS%d" % uart_id, baud)', "pinmap.set_pin_function"):
            self.assertIn(good, self.src, good)
        for bad in ("from machine import UART", "machine.UART(", "FPIOA.", "FPIOA("):
            self.assertNotIn(bad, self.src, "不要回退到庐山派 K230 的 %s" % bad)

    def test_servo_config(self):
        self.assertIn("servo", self.cfg)
        self.assertTrue(self.cfg["servo"]["enabled"])
        self.assertFalse(self.cfg["motor"]["enabled"])
        self.assertFalse(self.cfg["pitch_motor"]["enabled"])
        self.assertEqual(self.cfg["servo"]["yaw_id"], 0)
        self.assertEqual(self.cfg["servo"]["pitch_id"], 1)
        # MaixCAM2 引脚/端口
        self.assertEqual(self.cfg["servo"]["port"], 2)
        self.assertEqual(self.cfg["servo"]["tx_pin"], "B0")
        self.assertEqual(self.cfg["servo"]["rx_pin"], "B1")
        # 软件零点三键(不写舵机 EEPROM)
        for k in ("zero_auto", "yaw_zero_deg", "pitch_zero_deg"):
            self.assertIn(k, self.cfg["servo"], k)

    def test_servo_send_wired_to_pid(self):
        # 舵机路径必须用 servo_pid + servo_step_target, 不能退回纯 |e| 映射
        seg = self.src[self.src.index("def servo_send"):]
        seg = seg[:seg.index("def _servo_init")]
        self.assertIn("servo_pid", seg)
        self.assertIn("servo_step_target", seg)
        self.assertIn("kf_step", seg)


class _Clock:
    def __init__(self):
        self.t = 1000

    def ticks_ms(self):
        return self.t


class _Bus:
    def __init__(self):
        self.frames = []

    def send(self, f):
        self.frames.append(bytes(f))


class _Servo:
    def __init__(self, sid):
        self.id = sid
        self.calls = []

    def set_mturn(self, *a):
        self.calls.append(a)


class TestServoSendSim(unittest.TestCase):
    """把 main.py 的 servo_send + _servo_send_targets 放进 stub 环境跑闭环逻辑。"""

    def setUp(self):
        self.clock = _Clock()
        self.bus = _Bus()
        E.update({
            "time": self.clock,
            "_tdiff": lambda a, b: a - b,
            "_sbus": self.bus,
            "_syaw": _Servo(0),
            "_spitch": _Servo(1),
            "_s_ok_y": True,
            "_s_ok_p": True,
            "_s_yaw_target": 0.0,
            "_s_pitch_target": 0.0,
            "_s_yaw_i": 0.0,
            "_s_pitch_i": 0.0,
            "_s_yaw_e": 0.0,
            "_s_pitch_e": 0.0,
            "_s_last_ms": 0,
            "_s_mode": "idle",
            "_s_hold_t": 0,
            "_s_search_t": 0,
            "_s_search_dir": 1,
            "_s_kf_yaw": [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0],
            "_s_kf_pit": [0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0],
            "_s_off_y": 0.0, "_s_off_p": 0.0,
            "S_YAW_ID": 0, "S_PITCH_ID": 1,
            "S_YAW_SIGN": 1.0, "S_PITCH_SIGN": 1.0,
            "S_YAW_MIN": -150.0, "S_YAW_MAX": 150.0,
            "S_PITCH_MIN": -30.0, "S_PITCH_MAX": 30.0,
            "S_YAW_KP": 0.18, "S_YAW_KI": 0.08,
            "S_YAW_KD": 0.01, "S_YAW_KFF": 0.04,
            "S_PITCH_KP": 0.14, "S_PITCH_KI": 0.06,
            "S_PITCH_KD": 0.008, "S_PITCH_KFF": 0.03,
            "S_MAX_DPS": 150.0, "S_MAX_STEP": 6.0,
            "S_BOOST_PX": 0.0,
            "S_DEADBAND": 3.0, "S_IZONE": 60.0,
            "S_INTERVAL": 35, "S_POWER": 0, "S_SYNC": True,
            "S_HOLD_MS": 100, "S_SEARCH_EN": True,
            "S_SEARCH_DPS": 30.0, "S_SEARCH_RANGE": 120.0,
            "S_SEARCH_DIR": 1, "S_SEARCH_MAX_MS": 0,
            "S_KF_EN": False, "S_PID_EN": True,
            "S_KF_QPOS": 0.5, "S_KF_QVEL": 5.0, "S_KF_R": 2.0,
            "S_KF_LEAD": 0.0, "S_KF_LIM": 150,
        })

    def _call(self, dx, dy, status="track"):
        self.clock.t += 33
        E["servo_send"](dx, dy, 0, status)
        return E

    def _last_sync(self):
        self.assertTrue(self.bus.frames, "没有下发同步帧")
        f = self.bus.frames[-1]
        self.assertEqual(f[0:2], b"\x12\x4c")
        self.assertEqual(f[2], 0x19)
        c = f[4:-1]
        self.assertEqual(c[0], 0x0D)
        self.assertEqual(c[1], 0x0B)
        self.assertEqual(c[2], 2)
        out = []
        for i in range(2):
            o = 3 + i * 11
            out.append((c[o], E["fs_i32_at"](c, o + 1) * 0.1))
        return out

    def test_track_moves_target_and_frame_matches(self):
        self._call(100, 50)
        self.assertGreater(E["_s_yaw_target"], 0.0)
        self.assertGreater(E["_s_pitch_target"], 0.0)
        yaw, pitch = self._last_sync()
        self.assertEqual(yaw[0], 0)
        self.assertEqual(pitch[0], 1)
        self.assertLessEqual(abs(yaw[1] - E["_s_yaw_target"]), 0.0500001)
        self.assertLessEqual(abs(pitch[1] - E["_s_pitch_target"]), 0.0500001)

    def test_deadband_holds_target(self):
        self._call(100, 50)
        y0 = E["_s_yaw_target"]
        p0 = E["_s_pitch_target"]
        self._call(2, 1)
        self.assertAlmostEqual(E["_s_yaw_target"], y0)
        self.assertAlmostEqual(E["_s_pitch_target"], p0)

    def test_limit_clamps(self):
        E["_s_yaw_target"] = 149.0
        for _ in range(10):
            self._call(100, 0)
        self.assertLessEqual(E["_s_yaw_target"], 150.0 + 1e-9)

    def test_sign_flip(self):
        E["S_YAW_SIGN"] = -1.0
        self._call(100, 0)
        self.assertLess(E["_s_yaw_target"], 0.0)

    def test_lost_hold_then_search(self):
        self._call(0, 0, "lost")
        self.assertEqual(E["_s_mode"], "hold")
        self.clock.t += 120
        self._call(0, 0, "lost")
        self.assertEqual(E["_s_mode"], "search")

    def test_search_sweeps(self):
        self._call(0, 0, "search")
        self.assertEqual(E["_s_mode"], "search")
        t0 = E["_s_yaw_target"]
        for _ in range(5):
            self._call(0, 0, "search")
        self.assertNotAlmostEqual(E["_s_yaw_target"], t0)

    def test_soft_zero_offset_applied_to_frame(self):
        """软件零点: 帧里的角度 = 逻辑目标 + 偏移, 内部目标仍是逻辑角。"""
        E["_s_off_y"] = 60.0
        E["_s_off_p"] = -30.0
        self._call(100, 50)
        yaw, pitch = self._last_sync()
        self.assertAlmostEqual(yaw[1], E["_s_yaw_target"] + 60.0, delta=0.051)
        self.assertAlmostEqual(pitch[1], E["_s_pitch_target"] - 30.0, delta=0.051)
        self.assertLess(abs(E["_s_yaw_target"]), 30.0)   # 逻辑角, 没有被偏移污染

    def test_boot_send_applies_offset(self):
        E["_s_off_y"] = 60.0
        E["_s_off_p"] = -30.0
        E["_servo_boot_send"]([(0, 0.0), (1, 0.0)], 0)
        yaw, pitch = self._last_sync()
        self.assertAlmostEqual(yaw[1], 60.0, places=2)
        self.assertAlmostEqual(pitch[1], -30.0, places=2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
