"""采集端守门测试（MaixCAM2 / MaixPy 版）—— 防止采集/显示初始化被改回旧平台写法。

平台迁移（V11）后旧 K230-CanMV 的 `Sensor(id=CSI_ID)` / `set_framesize(crop=True)` /
1280x720 探测链已整条退役；MaixCAM2 的 `camera.Camera` 直接出 640x480（与板载 640x480
LCD 1:1），追踪中心 = 画面中心，显示 `disp.show(img)` 直显，不再做 crop / 软件缩放。

本测试（AST 静态检查 + config 读值, stdlib only, 不需要硬件）:
  1. main.py 恰好 1 处 `camera.Camera(...)`，参数为 `SENSOR_W, SENSOR_H, FMT_BGR888, fps=FPS`；
  2. `SENSOR_W/H` 从 config `camera.sensor_width/height` 读取，且为 640x480；
  3. `center.x/y` = 输出尺寸一半；`display.lcd_width/height` = 640x480（1:1 直显）；
  4. 显示只调用 `disp.show(img)` 一次；
  5. 不得回退旧平台 API（`Sensor(` / `set_framesize` / `CSI_ID`）；
  6. 不得引入已实机验证死亡的显示路径（cv2.resize / draw_image / image.Image /
     bind_layer / config_layer）。
若确需变更: 先实机确认方向/比例/视野，再同步改本测试，不要静默改掉守门。

运行: python tools/test_camera_init.py
"""
import ast
import json
import os
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_MAIN_PY = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "main.py"))
_CFG_JSON = os.path.normpath(os.path.join(_THIS_DIR, "..", "k230", "config.json"))


def _src():
    with open(_MAIN_PY, encoding="utf-8") as f:
        return f.read()


def _tree():
    return ast.parse(_src())


def _find_camera_calls(tree):
    calls = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "Camera"):
            calls.append(node)
    return calls


def _camera_call():
    calls = _find_camera_calls(_tree())
    assert len(calls) == 1, "k230/main.py 应恰好有 1 处 camera.Camera(...), 实际 %d 处" % len(calls)
    return calls[0]


def _find_disp_show_calls(tree):
    calls = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "show"):
            calls.append(node)
    return calls


class TestCameraInit(unittest.TestCase):
    def test_camera_call_exists_once(self):
        self.assertEqual(len(_find_camera_calls(_tree())), 1,
                         "k230/main.py 应恰好有 1 处 camera.Camera(...), 实际 %d 处"
                         % len(_find_camera_calls(_tree())))

    def test_camera_uses_sensor_dims(self):
        call = _camera_call()
        self.assertGreaterEqual(len(call.args), 2, "camera.Camera 至少传宽/高两个位置参数")
        names = [a.id if isinstance(a, ast.Name) else None for a in call.args[:2]]
        self.assertEqual(names, ["SENSOR_W", "SENSOR_H"],
                         "camera.Camera 前两个参数应为 SENSOR_W, SENSOR_H(从 config 读取)")

    def test_camera_format_bgr888(self):
        call = _camera_call()
        self.assertGreaterEqual(len(call.args), 3, "camera.Camera 缺少 format 参数")
        fmt = call.args[2]
        self.assertIsInstance(fmt, ast.Attribute, "format 应为 image.Format.FMT_BGR888")
        self.assertEqual(fmt.attr, "FMT_BGR888",
                         "MaixCAM2 采集格式应为 FMT_BGR888(cv2 零拷贝)")

    def test_camera_fps_from_config(self):
        call = _camera_call()
        kw = {k.arg: k.value for k in call.keywords}
        self.assertIn("fps", kw, "camera.Camera 应显式传 fps=FPS")
        self.assertIsInstance(kw["fps"], ast.Name)
        self.assertEqual(kw["fps"].id, "FPS", "fps 应取 config 的 FPS")

    def test_sensor_dims_defined_from_config(self):
        tree = _tree()
        defined = set()
        for node in tree.body:
            if isinstance(node, ast.Assign):
                for tgt in node.targets:
                    if isinstance(tgt, ast.Name):
                        defined.add(tgt.id)
        for n in ("SENSOR_W", "SENSOR_H", "FPS"):
            self.assertIn(n, defined, "k230/main.py 顶层必须定义 %s" % n)
        # DISPLAY_W/H 已删(未使用); disp.show 用屏幕自身尺寸, 不再读 config
        self.assertNotIn("DISPLAY_W", defined, "DISPLAY_W 是死代码, 不要再加回来")

    def test_config_camera_geometry(self):
        with open(_CFG_JSON, encoding="utf-8") as f:
            cfg = json.load(f)
        cam = cfg.get("camera", {})
        self.assertEqual(cam.get("sensor_width"), 640, "MaixCAM2 采集宽应为 640")
        self.assertEqual(cam.get("sensor_height"), 480, "MaixCAM2 采集高应为 480")
        self.assertGreater(cam.get("fps", 0), 0, "camera.fps 必须为正")
        self.assertNotIn("csi_id", cam, "MaixCAM2 不再使用 CSI_ID; 旧键应删除")

    def test_center_equals_half_output(self):
        with open(_CFG_JSON, encoding="utf-8") as f:
            cfg = json.load(f)
        cam, ctr = cfg.get("camera", {}), cfg.get("center", {})
        self.assertEqual(ctr.get("x"), cam.get("sensor_width") // 2,
                         "center.x 须 = sensor_width/2（改分辨率必须同步）")
        self.assertEqual(ctr.get("y"), cam.get("sensor_height") // 2,
                         "center.y 须 = sensor_height/2（改分辨率必须同步）")

    def test_display_matches_sensor(self):
        with open(_CFG_JSON, encoding="utf-8") as f:
            cfg = json.load(f)
        cam, disp = cfg.get("camera", {}), cfg.get("display", {})
        self.assertEqual(disp.get("lcd_width"), cam.get("sensor_width"),
                         "display.lcd_width 应 = sensor_width(1:1 直显)")
        self.assertEqual(disp.get("lcd_height"), cam.get("sensor_height"),
                         "display.lcd_height 应 = sensor_height(1:1 直显)")

    def test_display_show_once(self):
        calls = _find_disp_show_calls(_tree())
        self.assertEqual(len(calls), 1,
                         "k230/main.py 应恰好 1 处 disp.show(...)，实际 %d 处" % len(calls))


class TestNoLegacyCameraPaths(unittest.TestCase):
    def test_no_old_sensor_api(self):
        src = _src()
        for bad in ("Sensor(", "set_framesize", "CSI_ID", "crop=True"):
            self.assertNotIn(bad, src,
                             "k230/main.py 含旧平台 API %r；MaixCAM2 已改用 camera.Camera" % bad)


class TestNoDeadDisplayPaths(unittest.TestCase):
    def test_no_software_scale_paths(self):
        src = _src()
        for bad in ("cv2.resize", "draw_image", "image.Image",
                    "bind_layer", "config_layer"):
            self.assertNotIn(
                bad, src,
                "k230/main.py 含 %r —— 该显示路径实机不可用(花屏/卡死), 勿回退" % bad)


if __name__ == "__main__":
    unittest.main(verbosity=2)
