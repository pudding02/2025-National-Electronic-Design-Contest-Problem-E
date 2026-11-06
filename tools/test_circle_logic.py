"""画圈/搜索/几何纯函数主机侧测试（V10/V14 当前实现）。

历史: V8.5 的红环检测链（`classify_blob` / `ring_memory_step` / `calib_feed`）已在 V10
被删除——画圈改为「圆心=矩形中心, 半径按白框/圆物理尺寸比例推算」的纯几何主轨。
本测试因此不再测已删除函数，改为覆盖当前仍在生产路径上的纯函数:

  - circle_target_point     环心+半径+θ -> 像素点
  - circle_theta_step       θ 推进(误差门控/等待超时/方向/360° 环绕)
  - quad_area               四边形面积(shoelace)
  - search_step             上电搜索状态机
  - mode_pin_step           模式引脚去抖
  - geo_radius_px           V10 几何半径
  - estimate_distance_by_area / laser_position_by_distance  V9 测距/激光点映射

原理: AST 从 k230/main.py 提取函数源码, 在 stub 环境执行; 几何/距离常量从
k230/config.json 读取(与生产同一组数), 不接硬件、不依赖第三方库。

运行: python tools/test_circle_logic.py
"""
import ast
import json
import math
import os
import types
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_MAIN_PY = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "main.py"))
_CFG_JSON = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "config.json"))

_FUNC_NAMES = (
    "circle_target_point", "circle_theta_step", "quad_area", "search_step",
    "mode_pin_step", "geo_radius_px",
    "estimate_distance_by_area", "laser_position_by_distance",
)


class _FakeTime(types.ModuleType):
    def __init__(self):
        super().__init__("time")

    def ticks_diff(self, a, b):
        return a - b


def _load_env():
    with open(_MAIN_PY, encoding="utf-8") as f:
        src = f.read()
    with open(_CFG_JSON, encoding="utf-8") as f:
        cfg = json.load(f)
    c = cfg.get("circle", {})
    d = cfg.get("distance", {})
    env = {
        "math": math,
        "time": _FakeTime(),
        "_tdiff": lambda a, b: a - b,
        "RECT_W_CM": c.get("rect_w_cm", 0) or (c.get("paper_w_cm", 29.7)
                                               - 2.0 * c.get("tape_w_cm", 2.0)),
        "RECT_H_CM": c.get("rect_h_cm", 0) or (c.get("paper_h_cm", 21.0)
                                               - 2.0 * c.get("tape_w_cm", 2.0)),
        "CIRCLE_DIA_CM": c.get("circle_dia_cm", 11.3),
        "RADIUS_BIAS_CM": c.get("radius_bias_cm", 0.0),
        "DIST_REF_MM": d.get("ref_distance_mm", 1500),
        "DIST_REF_AREA": d.get("ref_area_px", 8412),
        "DIST_MIN": d.get("dist_min_mm", 500),
        "DIST_MAX": d.get("dist_max_mm", 1500),
        "LASER_X": d.get("laser_x", 322),
        "LASER_Y_NEAR": d.get("laser_y_near", 240),
        "LASER_Y_FAR": d.get("laser_y_far", 250),
    }
    tree = ast.parse(src)
    found = set()
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in _FUNC_NAMES:
            exec(ast.get_source_segment(src, node), env)
            found.add(node.name)
    missing = set(_FUNC_NAMES) - found
    assert not missing, "k230/main.py 缺函数: %s" % sorted(missing)
    return env


E = _load_env()


class TestCircleTargetPoint(unittest.TestCase):
    def test_zero(self):
        self.assertEqual(E["circle_target_point"](100, 100, 50, 0), (150, 100))

    def test_90(self):
        self.assertEqual(E["circle_target_point"](100, 100, 50, 90), (100, 150))

    def test_180(self):
        self.assertEqual(E["circle_target_point"](100, 100, 50, 180), (50, 100))

    def test_270(self):
        self.assertEqual(E["circle_target_point"](100, 100, 50, 270), (100, 50))

    def test_360_wraps_to_zero(self):
        self.assertEqual(E["circle_target_point"](100, 100, 50, 360), (150, 100))

    def test_rounding(self):
        # 45° -> 100 + 50*0.7071 = 135.36 -> 135
        self.assertEqual(E["circle_target_point"](100, 100, 50, 45), (135, 135))


class TestCircleThetaStep(unittest.TestCase):
    def test_dt_zero_noop(self):
        self.assertEqual(E["circle_theta_step"](10.0, 60, 0, 1, 0, 100, 0, 0),
                         (10.0, 0))

    def test_forward(self):
        th, w = E["circle_theta_step"](0.0, 60, 1000, 1, 0, 100, 0, 0)
        self.assertAlmostEqual(th, 60.0)
        self.assertEqual(w, 0)

    def test_backward_wrap(self):
        th, _ = E["circle_theta_step"](10.0, 60, 1000, -1, 0, 100, 0, 0)
        self.assertAlmostEqual(th, 310.0)

    def test_forward_wrap(self):
        th, _ = E["circle_theta_step"](350.0, 60, 1000, 1, 0, 100, 0, 0)
        self.assertAlmostEqual(th, 50.0)

    def test_pause_freezes_theta_and_accumulates_wait(self):
        th, w = E["circle_theta_step"](10.0, 60, 100, 1, 150, 100, 40, 0)
        self.assertAlmostEqual(th, 10.0)
        self.assertEqual(w, 140)

    def test_pause_timeout_resumes(self):
        # 已等 90ms, 本帧 20ms -> 110 >= wait_max 100, 继续推进
        th, w = E["circle_theta_step"](10.0, 60, 20, 1, 150, 100, 90, 100)
        self.assertAlmostEqual(th, 11.2)
        self.assertEqual(w, 110)

    def test_error_inside_gate_resets_wait(self):
        th, w = E["circle_theta_step"](10.0, 60, 100, 1, 50, 100, 500, 1000)
        self.assertAlmostEqual(th, 16.0)
        self.assertEqual(w, 0)


class TestQuadArea(unittest.TestCase):
    def test_square(self):
        self.assertEqual(E["quad_area"]([(0, 0), (4, 0), (4, 4), (0, 4)]), 16)

    def test_rectangle(self):
        self.assertEqual(E["quad_area"]([(0, 0), (8, 0), (8, 4), (0, 4)]), 32)

    def test_diamond(self):
        self.assertEqual(E["quad_area"]([(5, 0), (10, 5), (5, 10), (0, 5)]), 50)

    def test_degenerate_collinear(self):
        self.assertEqual(E["quad_area"]([(0, 0), (1, 1), (2, 2), (3, 3)]), 0)

    def test_vertex_rotation_invariant(self):
        self.assertEqual(E["quad_area"]([(4, 0), (4, 4), (0, 4), (0, 0)]), 16)


class TestSearchStep(unittest.TestCase):
    def test_disabled_idle(self):
        self.assertEqual(E["search_step"](1000, False, 0, False, False, 0),
                         ("idle", False, 0))

    def test_acquired_never_enters(self):
        self.assertEqual(E["search_step"](1000, True, 500, True, True, 0),
                         ("idle", False, 0))

    def test_start(self):
        self.assertEqual(E["search_step"](5000, False, 0, True, False, 0),
                         ("spin", True, 5000))

    def test_continue_keeps_start(self):
        self.assertEqual(E["search_step"](6000, True, 5000, True, False, 8000),
                         ("spin", True, 5000))

    def test_timeout_at_boundary(self):
        self.assertEqual(E["search_step"](13000, True, 5000, True, False, 8000),
                         ("timeout", False, 0))

    def test_no_timeout_when_unlimited(self):
        self.assertEqual(E["search_step"](99999999, True, 5000, True, False, 0),
                         ("spin", True, 5000))


class TestModePinStep(unittest.TestCase):
    def test_stable_keeps_value(self):
        self.assertEqual(E["mode_pin_step"](False, 0, False, 3), (False, 0))

    def test_debounce_first_mismatch(self):
        self.assertEqual(E["mode_pin_step"](False, 0, True, 3), (False, 1))

    def test_reaches_need_switches(self):
        self.assertEqual(E["mode_pin_step"](False, 2, True, 3), (True, 0))

    def test_bounce_resets_streak(self):
        self.assertEqual(E["mode_pin_step"](False, 2, False, 3), (False, 0))


class TestGeoRadius(unittest.TestCase):
    def _box(self, w, h):
        return [(0, 0), (w, 0), (w, h), (0, h)]

    def test_landscape(self):
        # 25.6x17cm 在 10px/cm -> s=10 -> 半径 10*11.3/2=56.5 -> 56
        self.assertEqual(E["geo_radius_px"](self._box(256, 170)), 56)

    def test_rotated_90(self):
        self.assertEqual(E["geo_radius_px"](self._box(170, 256)), 56)

    def test_zero_box(self):
        self.assertEqual(E["geo_radius_px"](self._box(0, 0)), 0)

    def test_config_matches(self):
        self.assertAlmostEqual(E["RECT_W_CM"], 25.6)
        self.assertAlmostEqual(E["RECT_H_CM"], 17.0)
        self.assertAlmostEqual(E["CIRCLE_DIA_CM"], 11.3)


class TestDistanceAndLaser(unittest.TestCase):
    def test_distance_at_ref(self):
        self.assertAlmostEqual(E["estimate_distance_by_area"](E["DIST_REF_AREA"]),
                               float(E["DIST_REF_MM"]))

    def test_distance_inverse_sqrt(self):
        area = E["DIST_REF_AREA"] / 4.0
        self.assertAlmostEqual(E["estimate_distance_by_area"](area),
                               E["DIST_REF_MM"] * 2.0)

    def test_distance_invalid_area(self):
        self.assertEqual(E["estimate_distance_by_area"](0), 0.0)

    def test_laser_near(self):
        self.assertEqual(E["laser_position_by_distance"](E["DIST_MIN"]),
                         (E["LASER_X"], E["LASER_Y_NEAR"]))

    def test_laser_far(self):
        self.assertEqual(E["laser_position_by_distance"](E["DIST_MAX"]),
                         (E["LASER_X"], E["LASER_Y_FAR"]))

    def test_laser_mid(self):
        mid = (E["DIST_MIN"] + E["DIST_MAX"]) // 2
        x, y = E["laser_position_by_distance"](mid)
        self.assertEqual(x, E["LASER_X"])
        self.assertAlmostEqual(y, (E["LASER_Y_NEAR"] + E["LASER_Y_FAR"]) / 2.0,
                               delta=1)

    def test_laser_clamps(self):
        self.assertEqual(E["laser_position_by_distance"](0),
                         (E["LASER_X"], E["LASER_Y_NEAR"]))
        self.assertEqual(E["laser_position_by_distance"](999999),
                         (E["LASER_X"], E["LASER_Y_FAR"]))


class TestStaticGuards(unittest.TestCase):
    def test_removed_ring_chain_not_reintroduced(self):
        with open(_MAIN_PY, encoding="utf-8") as f:
            src = f.read()
        for bad in ("def classify_blob", "def ring_memory_step", "def calib_feed"):
            self.assertNotIn(bad, src, "V10 已删除红环检测链, 勿回退: %s" % bad)

    def test_geometry_main_track_wired(self):
        with open(_MAIN_PY, encoding="utf-8") as f:
            src = f.read()
        for good in ("geo_radius_px", "circle_persp", "quad_area"):
            self.assertIn(good, src, "V14 几何主轨缺少 %s" % good)


if __name__ == "__main__":
    unittest.main(verbosity=2)
