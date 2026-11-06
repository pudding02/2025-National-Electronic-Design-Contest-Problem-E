---
name: "maixcam2-pinout"
description: "Sipeed MaixCAM2 开发板完整引脚总表与视觉云台项目接线速查(UART2/UART3 双路 Emm42 步进、A4 模式脚、禁用引脚警示、全部丝印引脚的功能复用)。当需要查询 MaixCAM2 引脚定义、串口分配、接线方式、供电约束、引脚复用(pinmap)时调用。K230/庐山派时代的接线已整板废弃, 以本文件为准。"
---

# Sipeed MaixCAM2 引脚图与项目接线速查

> 适用板卡：Sipeed MaixCAM2（MaixPy v4 固件，板载 640×480 触摸 LCD）。
> 前身 K230（庐山派 CanMV）整板替换，旧接线文档 `LUSHAN-PI-K230-PINOUT-SKILL.md` 仅作历史参考。
> 引脚名一律用**板上丝印名**（A4/B0/B2…），MaixPy API 与实物丝印一致，不再用 GPIO 编号。
> 自查命令：设备上跑 `pinmap.get_pins()` 可列出全部可用引脚（板子自报，优先级高于任何文档）。

---

## 〇、完整引脚总表（板载丝印 v1.0，功能复用用 `/` 分隔）

> 电源/地不占用 IO 编号；所有 GPIO 为 **3.3V 电平，严禁直接接入 5V 信号**。

### 1. 左侧 2×6P 2.54mm 排针（上→下）

| # | 功能 | IO 编号 |
| --- | --- | --- |
| 1 | 3V3（电源输出） | — |
| 2 | GND | — |
| 3 | UART2_TX | B0 |
| 4 | UART2_RX | B1 |
| 5 | I2C7_SCL | A8 |
| 6 | I2C7_SDA | A9 |
| 7 | SPI2_CS1 | B21 |
| 8 | SPI2_MOSI | B18 |
| 9 | SPI2_MISO | B19 |
| 10 | SPI2_SCK | B20 |
| 11 | GND | — |
| 12 | 3V3（电源输出） | — |

### 2. 左下 6P 1.25mm 接口（上→下）

| # | 功能 | IO 编号 |
| --- | --- | --- |
| 1 | GND | — |
| 2 | 5V（电源输入 3.7~5V） | — |
| 3 | SPI1_MISO / I2C6_SCL | A1 |
| 4 | SPI1_MOSI / I2C6_SDA | A0 |
| 5 | UART4_TX | A21 |
| 6 | UART4_RX | A22 |

### 3. 右侧 2×6P 2.54mm 排针（上→下）

| # | 功能 | IO 编号 |
| --- | --- | --- |
| 1 | GND | — |
| 2 | 5V（电源输入 3.7~5V） | — |
| 3 | PWM7 / UART1_RX | A31 |
| 4 | PWM4 / UART3_TX / SPI2_MOSI | B2 |
| 5 | UART0_TX | A28 |
| 6 | SPI1_SCK | A4 |
| 7 | SPI1_CS0 | A2 |
| 8 | UART0_RX | A29 |
| 9 | PWM5 / UART3_RX / SPI2_MISO | B3 |
| 10 | PWM6 / UART1_TX / SPI2_CS3 | A30 |
| 11 | 5V（电源输入 3.7~5V） | — |
| 12 | GND | — |

### 4. 板载固定 IO

| 外设 | IO 编号 | 说明 |
| --- | --- | --- |
| 补光灯 Light (PWM6) | B25 | 可 PWM 调亮度 |
| func/boot | C2 | 启动模式选择脚，业务代码勿占用 |
| State_LED 状态灯 | A6 | 系统状态灯，可复用为 GPIO 输出 |
| Power_LED 电源灯 | 独立硬件 | 软件不可控 |

### 5. 供电约束

| 来源 | 5V 引脚电压 |
| --- | --- |
| Type-C / 5V pin | 输入 3.7~5V |
| 锂电池单独供电 | 3.7~4.2V |

### 6. 板载硬件外设（非 IO）

M2×8 内六角固定螺丝孔、M12×0.5 镜头座（基座高度 12/14.5mm）、电源开关、板载喇叭 Speaker、Type-C (USB2.0)、MIC_R/MIC_L 双麦克风、TF(Micro-SD) 卡座、6P FPC 内置 100M ePHY→RJ45 网口、2.4 寸 640×480 触摸屏 FPC 口、1/4" 相机标准螺孔。

### 7. 引脚复用示例（MaixPy v4）

```python
from maix import pinmap
pinmap.set_pin_function("B2", "UART3_TX")   # 把 B2 配置为 UART3 发送
```

---

## 一、串口资源分配（实测确认）

| 串口 | 设备节点 | 引脚(丝印) | 状态 | 用途 |
| --- | --- | --- | --- | --- |
| UART2 | /dev/ttyS2 | B0=TX, B1=RX（左排针） | **本项目占用** | yaw 轴 Emm42 步进驱动 |
| UART3 | /dev/ttyS3 | B2=TX, B3=RX（右排针） | **本项目占用** | pitch 轴 Emm42 步进驱动 |
| UART0 | /dev/ttyS0 | A28=TX, A29=RX | **禁接** | 系统日志口，占用后丢日志 |
| UART4 | /dev/ttyS4 | A21=TX, A22=RX | **禁接** | Maix Comm Protocol 自动占用，程序启动即打印 `auto set pinmap A21/A22`，任何程序都无法抢用 |

## 二、本项目实际接线

### 1. 双路 Emm42 步进驱动器（115200, 8N1, 帧头 0xFE / 校验 0x6B, 大端）

```
MaixCAM2                          Emm42-yaw (ID对应 yaw)
  B0  TX ──────────[470Ω]──────►  RXD
  B1  RX ◄──────────────────────  TXD
  GND ──────────────────────────  GND   (必须共地)

MaixCAM2                          Emm42-pitch
  B2  TX ──────────[470Ω]──────►  RXD
  B3  RX ◄──────────────────────  TXD
  GND ──────────────────────────  GND
```

- TX 线串阻教训：1kΩ 导致信号衰减、应答帧不完整（`read_bus_voltage` 频繁 `(no resp)`）→ 改 **470Ω 或 330Ω**，距离短可直连。
- RX 线不串阻。两路独立串口，无多机总线，Emm42 的 ID 只用于帧内寻址。

### 2. 模式跳线（可选备用，默认触屏 T/C 切换）

```
MaixCAM2 3V3 ──[跳线]──► A4 (右侧排针, SPI1_SCK 丝印位)
```

- A4 被供电 = 画圈模式；拨 GND/悬空 = 仅追踪。config `[mode_pin].enabled=true` 时启用，默认 false（触屏切换）。

### 3. 供电（硬约束）

| 设备 | 供电方式 | 禁止事项 |
| --- | --- | --- |
| MaixCAM2 | Type-C 5V（调试时连电脑） | — |
| Emm42 驱动器 ×2 | UC-01 载板 DC 口 6~12.6V | **不得**从 MaixCAM2 取电 |
| 云台运行时 | — | **Type-C 不得连接电脑**（防 USB 与驱动器 TX 总线冲突） |

### 4. 触摸屏与显示

板载 640×480 触摸 LCD，无需接线。采集与显示同分辨率 640×480 时触点映射为恒等（`image.resize_map_pos_reverse` 仍保留调用，黑边场景自动生效）。日志里 `Win-info: 480x640` 是 VO 内部缓冲尺寸，与触摸坐标无关，勿据此调接线。

---

## 三、K230 → MaixCAM2 接线变更对照

| 项 | K230（废弃） | MaixCAM2（现行） |
| --- | --- | --- |
| yaw 串口 | UART2 (GPIO5/6, Pin11/13) + FPIOA | UART2 (B0/B1)，pinmap.set_pin_function |
| pitch 串口 | 旧 UART（GH1.25-4p） | UART3 (B2/B3) |
| 模式脚 | A4 (FPIOA) | A4（gpio API，丝印名直用） |
| TB6612 电机驱动 | GPIO23/20/25 + STBY 硬线 3.3V | 已随整板迁移废弃 |
| 触摸 | machine.TOUCH | touchscreen.TouchScreen（内置 LCD） |
