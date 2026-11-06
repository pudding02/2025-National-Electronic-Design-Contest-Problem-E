---
name: "maixcam2-maixpy"
description: "MaixCAM2/MaixPy v4 平台开发要点与踩坑实录（2026-10 视觉云台 K230→MaixCAM2 迁移全程实证）。涵盖 ticks_diff 符号陷阱、MaixVision 文件重载与上传机制、触摸事件模型(丢沿/排空/看门狗)、UART/pinmap、透视单应画圆、调试方法论。写 MaixPy 代码前必读, 尤其是从 K230 CanMV 迁移过来的项目。"
---

# MaixCAM2 / MaixPy v4 平台开发要点（项目实证）

> 来源：视觉云台项目（`k230/main.py`）从 K230 CanMV 整板迁移到 MaixCAM2 的实测记录。
> 所有结论均在真机取证过，标注了证据。接线见 `maixcan2引脚图.md`。

---

## 一、致命坑（按杀伤力排序，先读这节）

### 1. `time.ticks_diff` 符号与 MicroPython 标准相反【最隐蔽, 必炸】

MaixPy 真机上 `time.ticks_diff(a, b)` 返回 **b−a**（标准 MicroPython 语义是 a−b）。

**实测铁证**（触摸防抖日志）：`now=3892890 act_t=3886984`，真实差 +5906，`ticks_diff` 返回 -5906；且三组样本的差值 2125/2477 与 now 增量严格吻合——不是回绕，是恒定反号。

连锁症状：防抖 dt 恒负→按钮只能点一次；超时判断永不满足→状态机冻结；连发永不触发；dt 恒负还会让速度积分反向。

**修法**：全文弃用，用纯算术减法（`ticks_ms` 为 64 位单调毫秒，运行期无回绕）：

```python
def _tdiff(a, b):
    """毫秒差 a-b。MaixPy 真机 ticks_diff 符号与 MicroPython 相反(实测),
    弃用之直接算术相减。"""
    return a - b
```

**检测技巧**：任何"应该为正的时间差"打印出负值，先怀疑本坑，把 `now` 与基准的原始值一起打出来对比量级。

### 2. MaixVision 不重载磁盘上被外部修改的文件【浪费 3 轮排查的元凶】

MaixVision **上传运行的是编辑器内存里的内容**。main.py 标签页一直开着时，磁盘上被外部工具修改的代码不会自动重载——点「运行」传上去的还是打开时的旧快照，且无任何提示。

**纪律**：外部改完代码 → MaixVision 里**关闭标签页重新打开**（保险起见重启 IDE）→ 确认能看到新代码的特征字符串 → 再运行。

**版本指纹法**：在启动日志里打印一行包含新改动特征值的配置快照（如 `circ cfg: speed=60.0 ... persp=on`），上机后日志有这行且值正确 = 新代码确实上板。不要靠"现象对不对"判断版本。

### 3. 「运行项目」≠「运行当前文件」

- **运行项目**：上传整个文件夹到 `/tmp/maixpy_run/`，config.json 等资源文件可用
- **运行当前文件**：只传单个 .py，config.json 缺失 → 程序静默用默认值（踩过：日志 `config: using defaults`）

含资源文件的项目一律用「运行项目」。config 加载候选路径里应包含脚本目录 `os.path.dirname(abspath(__file__))` 优先 + `/root/config.json` 兜底，每个失败候选打印原因。

### 4. 触摸是事件驱动模型，不是电平查询

MaixPy `TouchScreen.read()` 读内部事件缓冲的**缓存状态**；无事件时返回上一次状态。视觉主循环帧间隔长（尤其 640×480@60 带宽不足掉帧时）时，一次"按下-松开-再按下"被压进同一帧，单次 read 只看到连续按下、**丢掉中间松开沿**——表现为"按钮只能点一次"。

**修法四件套**（`k230/main.py` 触摸段，全部实证有效）：

```python
# ① 每帧排空积压事件, 捕捉"松开后再按下"的缝隙
_have_evt = tp.available(0)
tx, ty, pressed = tp.read()
gap = False
while ...:
    if not tp.available(0): break
    nx, ny, np_ = tp.read0()
    if np_ and not pressed: gap = True   # False->True: 一次新按压
    pressed = np_; ...
# ② 缝隙补沿: 本批事件含重按 -> 强制出 F->T 沿
was = False if (pressed and gap) else pre
# ③ 静止看门狗: 按住却 800ms 无任何事件(正常按住手指微动会产生 MOVE),
#    判定固件丢了抬手 -> 本帧强制松开, 下次真按压自动恢复
# ④ read 异常只当本帧未按下+限频打印, 绝不永久禁用触摸(旧逻辑一次异常触屏报废)
```

### 5. 防抖基准用"最近动作时刻"而非"按住起始时刻"

按住起始时刻 `hold_t0` 同时承担连发计时、被多处透传/刷新，语义污染后防抖会误吞正常连点。防抖本质是"同一按钮两次**动作**的间隔"，独立维护 `act_t`（只在发动作时写 now），与连发计时彻底分家。

### 6. pinmap API 返回 Err 码，不抛异常

`pinmap.set_pin_function("B0", "UART2_TX")` 失败时**返回错误码**，不抛异常——必须检查返回值，否则静默失败。引脚名用板上丝印字符串（"A4"/"B0"），用 `pinmap.get_pins()` 自报核实，不要凭文档记忆。

---

## 二、K230 CanMV → MaixPy API 映射（已验证写法）

| 功能 | K230 CanMV（废弃） | MaixPy v4（现行） |
| --- | --- | --- |
| 摄像头 | Sensor + FPIOA 构造 | `camera.Camera(W, H, image.Format.FMT_BGR888, fps=60)` + `skip_frames(10)` |
| 取帧到 cv2 | Sensor snapshot + 手工转换 | `img = cam.read()`; `frame = image.image2cv(img, ensure_bgr=False, copy=False)`（零拷贝, cv2 直改帧存） |
| 显示 | Display + show_image | `disp = display.Display()`; `disp.show(img)`（默认 FIT_CONTAIN） |
| 触摸 | machine.TOUCH | `ts = touchscreen.TouchScreen()`; `x, y, p = ts.read()`（事件模型, 见坑 4） |
| 串口 | machine.UART + FPIOA | `pinmap.set_pin_function(tx, "UART2_TX")`; `uart.UART("/dev/ttyS2", 115200)` |
| 串口收 | `uart.any()` + read | **无 `any()`**：`read(n, 0)` / `read(-1, 0)` 非阻塞（超时 0）；裸 `read()` 不永久阻塞（探针实证） |
| GPIO | machine.Pin + FPIOA | `gpio.GPIO("A4", gpio.Mode.IN, gpio.PULL_NONE)`; `.value()` |
| 主循环 | `while True` + `exitpoint()` | `while not app.need_exit():`（IDE 才能正常停止进程） |
| 分辨率 | sensor 固定出图比例 | Camera 直接出目标尺寸；宽高比与传感器(16:9)不符时**居中裁切**——640×480(4:3) 会收窄水平视野，视野敏感参数需重标定 |
| numpy | 需确认 | 可用（cv2 依赖自带），`import numpy as np` |

**屏幕/触摸坐标系**：`disp.width()/height()` 为屏幕分辨率；触摸点映射回图像系用 `image.resize_map_pos_reverse(IMG_W, IMG_H, SCR_W, SCR_H, image.Fit.FIT_CONTAIN, x, y)`。采集=屏幕分辨率时映射为恒等。VO 日志的 `480x640` 是内部缓冲尺寸，与触摸无关。

---

## 三、调试方法论（本次实战沉淀）

1. **探针先行**：硬件行为不确定时，先用 `main.py` 的启动日志（`actuator:` / `att:` / `calib:` 三行）逐段取证，**不要凭推断改产品代码**——本项目连续 3 个错误推断（裸 read 阻塞、UART2 写不通、配置路径）全被实机日志证伪。
2. **诊断日志三要素**：按下/松开沿打印 + 动作发出打印（`touch: fire mode now=%d`）+ 被吞打印（`touch: guarded ... dt=%d (now=%d act_t=%d)`），一次上机定位整条链路；高频日志限频（500ms tick）。
3. **原始值一起打**：怀疑时间戳问题时打 `dt` 的同时打 `now` 与基准的原始值——差值符号对不上量的那一眼就是根因（坑 1 就是这样抓到的）。
4. **主机侧纯函数测试**：把几何/状态机写成不碰硬件的纯函数，测试脚本用 AST 从 main.py 提取函数源码在 stub 环境断言（`tools/test_circle_logic.py` 模式），不依赖硬件也不依赖第三方库。

## 四、透视单应画圆（矩形→圆的透视一致）

需求：靶纸矩形在画面里是透视四边形时，画的圆要跟着变形（透视椭圆）。

**方法**：检测轮廓 `approxPolyDP` 取 4 角点（`order_points` 环形序 TL,TR,BR,BL）→ 与物理矩形（±W/2, ±H/2, 物理系 y 向下与图像一致）建 `cv2.getPerspectiveTransform` → 物理圆 72 个采样点 `cv2.perspectiveTransform` 投影成折线（`cv2.polylines` isClosed=True 绘制）→ 黄点走同一矩阵的单点投影闭包。物理半径 = 直径/2 + bias_cm，配置非法/矩阵退化/投影发散时降级正圆推算（日志前缀区分 `circ:` 透视 / `circ(2d):` 降级）。

**注意**：物理矩形宽高比若与画面中检测框比例不符（尺寸没实测准），投影圆会变成椭圆——几何上是正确行为，提示去实测修正 `rect_w_cm/rect_h_cm`。函数参考 `main.py` 的 `circle_persp()` / `circle_polyline()`。
