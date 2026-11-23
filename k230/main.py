"""K230 主机程序 2026-10-19 版（YOLO 网格 + 轨迹 + UART2 控制器）。
K230 V3.0 / CanMV 1.2.2：YOLO26n 网格锁定、轨迹测量与要求5控制器，
通过 UART2（GPIO5/GPIO6，230400 8N1）通信。
需将 best.kmodel 上传到 /sdcard/grid_yolo/，并在 CanMV IDE 中运行本文件。
K230 是主机：接收 F407 发来的 $START,shape[,freq]，回发 $SET/$LOCKED/$DONE/$FAILED
freq=装置自身当前输出频率（mHz，仅作扫频起点，非外部传入的信号源频率点）
（与 PC 主机相同的 ASCII 协议）。
"""
import os
import sys
if "/sdcard/grid_yolo" not in sys.path:
    sys.path.insert(0,"/sdcard/grid_yolo")
import gc
import time
try:
    import ujson
except ImportError:
    import json as ujson  # PC 回归测试环境无 ujson，API 兼容。
try:
    import nncase_runtime as nn
except ImportError:
    nn = None
try:
    import ulab.numpy as np
except ImportError:
    np = None
"""CanMV K230 要求5李萨如视觉控制器。

刻意做成单文件部署：复制到 K230 的 ``/sd/main.py`` 即可。
硬件导入加了保护，频率运算、形状评分与控制器也可被 CPython 回归测试导入。

K230 是主机，绝不电气采样未知信号源；X 信号的全部信息来自示波器黄色轨迹。
F407 只是用下方文档描述的 ASCII 协议执行 DDS 输出。
"""

import gc
import math
import os
import time

# 仅 CanMV 模块在主机端测试时可省略。
try:
    from machine import UART, FPIOA
    from media.sensor import Sensor
    from media.display import Display
    from media.media import MediaManager
    CANMV_AVAILABLE = True
except (ImportError, OSError):
    UART = None
    FPIOA = None
    Sensor = None
    Display = None
    MediaManager = None
    CANMV_AVAILABLE = False

# ---------------------------------------------------------------------------
# 摄像头、ROI 与现场标定常量
# ---------------------------------------------------------------------------

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 320

# K230 V3.0 板显示配置。
# 摄像头帧为 640x480，板上 ST7701 屏幕为 800x480。
# 两者尺寸必须分开：若对 ST7701 使用 640x480，部分 CanMV 固件版本
# 会把硬件显示置为未选中/配置错误。
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
DISPLAY_TARGET = "lcd"       # "lcd"、"virt" 或 "auto"
USE_LCD = DISPLAY_TARGET == "lcd"
DISPLAY_FPS = 60              # VIRT 使用；ST7701 用板默认值。
ROTATE_IMAGE_180 = True        # 摄像头在板上倒装。
ACTIVE_PREVIEW_INTERVAL_FRAMES = 3
LOG_INTERVAL_FRAMES = 90
# D11 修复(2026-10-22)：Sensor 构造显式指定 fps（默认 30；GC2093 支持更高，
# 社区实测 OV5647 1080P 仅 30fps，要更高帧率需按 sensor 型号在 config.json
# 设 sensor_fps）。K230 上 640x480 双通道带宽是帧率实际瓶颈。
SENSOR_FPS = 30

# 覆盖 8x8 div 屏幕区的网格矩形（640x480 摄像头帧坐标）。
# 2026-10-22 修复：旧值 (44,12,228,287) 是从本框按轴缩放出的 320x320 版本，
# 但运行时 roi 直接作用于 640x480 的 chn0 帧（YOLO 失锁回退路径），
# 会把 ROI 落到左上角一小块并让 div 换算缩水一半。这里直接用现场标定值；
# 若以后把视觉输入改成 320x320，必须整体乘 (0.5, 2/3) 再改。
# FIXME(真机): 仍须用绿色叠加层复核四边是否贴合网格。
GRID_RECT = (89, 18, 456, 431)
# 网格为 8x8 div。GRID_DIVISIONS_X 原先为 10.0，使所有 X-span/相位数值
# 缩水 0.8 倍（456px/8div=57px/div，而非 45.6）。
GRID_DIVISIONS_X = 8.0
GRID_DIVISIONS_Y = 8.0

# 继承自可用黄色轨迹实现的 LAB 阈值。
# CanMV LAB 顺序为 (Lmin, Lmax, Amin, Amax, Bmin, Bmax)。
YELLOW_THRESHOLD = (40, 100, -45, 25, 15, 100)
# FPS-OPT: 2 -> 3。ROI 内逐像素采样点约由 16k 降到 7k，
# Python 遍历耗时约减半；波形为连续曲线，投影/形状评分精度不受影响。
TRACE_SAMPLE_STEP = 3
MASK_BUFFER_ENABLED = True
MASK_ACCESS_MODE = 'unchecked'
MIN_TRACE_POINTS = 120
MIN_TRACE_SPAN_DIV = 2.0
TRIM_FRACTION = 0.002
# 2026-10-22：削顶/出屏检测。轨迹以陡角穿过 ROI 边缘（实际幅度超出
# 屏幕）时，边缘带内的点会沿边分布很远；恰满幅（相切/过角点）时只
# 集中在一个很小的区间。用于识别"看上去仍是 8div 但实际已过幅"的
# 圆/∞ 形。对角线过幅在屏内与恰满幅完全重合，此指标无法识别。
# 合成圆实测：恰好 8div（相切）边缘跨度 1.52div；过幅 5% -> 2.92div，
# 25% -> 6.2div。门限 2.5div 可抓 >=5% 过幅，且与相切工况留 1div 余量。
EDGE_CUT_BAND_PX = 2.0
EDGE_CUT_SPREAD_DIV = 2.5

# 四条边缘带是刻意独立的测量。某边只有在至少三个带宽返回相同峰值计数时
# 才被接受。
EDGE_BAND_RATIOS = (0.010, 0.015, 0.020, 0.025)
MULTISCALE_AGREE_COUNT = 3
PEAK_MIN_DISTANCE_PX = 8
CONTACT_POSITION_TOLERANCE_PX = 4
MAX_TOUCH_COUNT = 5

# ---------------------------------------------------------------------------
# 快速搜索预过滤器（填充率静态门限）
# ---------------------------------------------------------------------------
# 轨迹只有渲染成细线时才“静态”（接近小整数比）：fill_ratio = 黄色像素/包围盒面积，
# 值低。当探针频率远离未知输入的小整数比时，示波器余晖画出旋转包络，填充率饱和偏高。
# 搜索因此跳过单帧填充率超过门限的探针，只对少数静态候选跑昂贵的峰值计数流水线。
FAST_BLOB_PIXELS_THRESHOLD = 16
# 快速预判门限：counts/span 模式下 fill 高于此值 = 满格旋转包络
# （远离小整数频率比），直接返回快速帧跳过 Python 遍历与四边投影，
# 把满格档分析从 ~170ms 压到一次 find_blobs(~15ms)。
# 真机标定项：介于快速门限(0.30) 与静态满格图实测 fill 之间取中。
FAST_FILL_THRESHOLD = 0.30
# X 幅度测量兜底帧数：5 帧收敛条件在曝光/噪声漂移下可能永不满足，
# 超过该帧数后按已有采样中位数直接进入搜索，避免拖到 10 s 超时。
MEASURE_X_MAX_FRAMES = 25

# 471mV 指令经模拟链路 x8.5 放大 = 4.0Vpp = 8div @0.5V/div（用户 2026-10-22 确认）。
# 这些条目是校准锚点，不是对未知输入的假设；模拟链路衰减档变化时用台架实测
# 替换指令值即可，插值自动进行。注意：表值只对 1:1 输出链路有效，2:1（∞）
# 档的幅度闭环会按实测 span 自行修正。
DDS_REFERENCE_SPAN_DIV = 8.0
DDS_AMPLITUDE_TABLE = (
    (10, 471),       # 1 kHz
    (50, 471),       # 5 kHz
    (100, 471),      # 10 kHz
    (200, 471),      # 20 kHz
    (500, 471),      # 50 kHz
    (1000, 471),     # 100 kHz
    (1500, 471),     # 150 kHz
    (2000, 471),     # 200 kHz
)
DDS_AMPLITUDE_MIN_MV = 20
DDS_AMPLITUDE_MAX_MV = 500

# D9 修复(2026-10-22)：X 幅度 Vpp 换算的示波器 Y 增益配置化。
# 现场必须把示波器 Y 通道锁定在 VOLTS_PER_DIV V/div（默认 0.5），
# 与 DDS 链路标定（471mV 指令 x8.5 放大 = 4.0V = 8div @0.5V/div）自洽。
VOLTS_PER_DIV = 0.5

# ---------------------------------------------------------------------------
# UART2 与协议常量
# ---------------------------------------------------------------------------

UART2_TX_PIN = 5
UART2_RX_PIN = 6
UART2_BAUDRATE = 230400
UART_ACK_TIMEOUT_MS = 80
UART_MAX_ACK_RETRIES = 1
DISCARD_FRAMES_AFTER_APPLIED = 1
# The camera runs at about 30 fps.  A 30 ms delay can therefore leave the
# first analysed frame inside the DAC/relay transition.  Keep one full frame
# of settling before the RAMP estimator starts consuming samples.
SWEEP_POST_APPLIED_SETTLE_MS = 100

CONTROL_TIMEOUT_MS = 10000
HOLD_REQUIRED_MS = 5000

# ---------------------------------------------------------------------------
# 频率档位范围（grid = 频率/100Hz；1kHz~100kHz → grid ∈ [10, 1000]）。
# 2026-10-22 按用户明确范围：1kHz~100kHz、步进 100Hz（grid 上限 1000）。
SWEEP_GRID_MIN = 10               # 1kHz -> grid 10
SWEEP_GRID_MAX = 1000             # 1:1 输出上限：100kHz -> grid 1000
# 2026-10-22：∞ 形是 2:1，输入 100kHz 时装置必须输出 200kHz。旧实现
# 复用 SWEEP_GRID_MAX，输入 >50kHz 时被硬夹到 100kHz，2:1 图形直接画不出来。
OUTPUT_GRID_MAX_2TO1 = 2000       # 2:1 输出上限：200kHz -> grid 2000
ROTATION_HZ_PER_TURN_DEG = 180.0  # 主轴角速度(°/s) -> Δf(Hz)，1:1 标定值
# D10 修复(2026-10-22)：Δf 小时 5 帧(≈160ms)角速度噪声主导。
# 若算出的 |Δf|<0.3Hz 且时间窗<300ms，判定噪声不可信，继续收集角度
# 样本延长观测窗（最多 ROTATION_MAX_SAMPLES 帧兜底）。
ROTATION_MIN_DELTA_HZ = 0.3
ROTATION_MIN_WINDOW_MS = 300
ROTATION_MAX_SAMPLES = 10
# D4 修复(2026-10-22)：旋转方向符号做成 config 可配（真机 +1Hz 探针标定）。
# sign=+1 保持"主轴正向旋转(角度递增)=fy>fx -> 减频"原语义；标定发现
# 方向反了在 config.json 设 "rotation_sign": -1。
ROTATION_SIGN = 1
# D4 修复：2:1 目标（∞ 形）旋转速率与 Δf 的比例与 1:1 不同，独立标定项；
# 未真机标定前默认沿用 1:1 值，真机实测后在 config.json 覆盖。
ROTATION_HZ_PER_TURN_DEG_2TO1 = 180.0
ROTATION_SAMPLES = 5              # ROTATION_LOCK 主轴角样本数
ROTATION_MAX_ROUNDS = 4           # 最多跳频轮数（残余漂移由 HOLD 兜底）
PHASE_FINE_GOOD_FRAMES = 3        # PHASE_FINE 形状 pass 连续帧数 -> 收敛
PHASE_SCAN_STEP_DEG = 30          # 相位粗扫覆盖完整周期
PHASE_REFINE_STEP_DEG = 5         # 最优粗相位附近的细扫步长
# 2026-10-22：幅度闭环容差。注意形状 pass 门限（对角线/圆/∞ 的
# span_difference<=0.12）隐含要求 X/Y 两轴跨度一致；若器件 Y 幅度偏离
# 目标超过该值，PHASE_FINE 永远等不到 pass，也进不了 AMPLITUDE_FINE。
# 因此进入相位扫描前先用与相位无关的 span_y 做一次幅度预收敛。
AMPLITUDE_CONVERGE_DIV = 0.12
HOLD_MAX_SAMPLE_GAP_MS = 250      # 超过此间隔不能算作连续稳定观测
HOLD_FILL_DRIFT_GATE = 0.10       # HOLD 期间 fill 相对参考的最大上升（漂移门限）
HOLD_SPAN_MIN_DIV = 2.0           # HOLD 期间图形存在的最小 span（div）
# 2026-10-22：赛题要求器件幅度误差 <=0.2div。旧实现用统一 ±1.0div 的
# "相对 LOCKED 时刻漂移" 门限，幅度在 5s 内漂 0.9div 仍算稳定。改为
# span_y 始终相对目标值检查；中心只要求不跑偏。
HOLD_SPAN_DRIFT_DIV = 0.20        # HOLD 期间 span_y 相对目标的容差
HOLD_Y_REFERENCE_TOL_DIV = 0.25   # HOLD 期间源 X span 相对参考的容差
HOLD_CENTER_DRIFT_DIV = 1.0       # HOLD 期间图形中心允许的漂移

# ---------------------------------------------------------------------------
# v3：固定周期短斜坡粗测频（RAMP_ESTIMATE）
# 依据 2026F题_网上调研/RAMP斜坡测频方案_可行性分析.md 与公开同源实现
# CdyWork/2026F-lissajous-controller（q5_sawtooth_vision_sim）：
# Y 通道输出已知周期的锯齿斜坡（F407 DAC PA4），XY 显示把 X 输入正弦沿
# Y 轴展开成时间波形 x(u)=A·sin(2π·N·u)，N=fx·T_ramp，fx=N/T_ramp。
# 单帧内行质心 + 整数周期 DFT 粗估 + 最小二乘正弦拟合即可估频，不需要
# 多帧叠加；斜坡重复周期与 100Hz 网格对齐时多帧叠加不糊（10ms=100Hz
# 周期整数倍，每次斜坡起点输入相位一致）。
# 硬件前提：PA4 已接示波器 Y 通道（当前 AD9959 是 Y 源，需接线/切换）。
# ---------------------------------------------------------------------------
RAMP_ENABLED = True               # 主开关：False 时完全走 v2 链路
RAMP_SAW_HZ = 1000                # 初始斜坡频率（1ms 重复周期）
RAMP_AMP_MV = 3300                # 斜坡幅度（F407 DAC 满幅 3.3V）
# 2026-10-22：1 -> 1。赛题下限 1kHz，初始 1kHz 斜坡下 N=1；合成轨迹实测
# N=1 时估计 1.002、质量 0.997，完全可用；N=0.5 时 0.5->0.56（12% 误差），
# 因此下限取 1 而不是更小。旧值 4 会把 1kHz 输入判成越界并向下换档，
# 换到 500/250Hz 后 N 更小，最终必然 FAILED。
RAMP_MIN_CYCLES = 1               # 斜坡窗口内最小可接受周期数
RAMP_MAX_CYCLES = 60              # 斜坡窗口内最大可接受周期数
RAMP_MAX_ITERATIONS = 5           # 换斜坡档最多迭代次数
RAMP_SHOT_MAX_FRAMES = 5          # 单档斜坡最多采集帧数
# D1 修复(2026-10-22)：质量门限 0.25 -> 0.50。合成轨迹实证：
# 真实 N>=100（1kHz 斜坡对应 >=100kHz 输入）时 DFT 欠采样混叠，
# 泄漏估计质量恒在 0.29 附近，会被 0.25 门限放行导致锁错频率；
# 0.50 门限把所有混叠估计拦成失败帧 -> 自动换快档重测。
RAMP_QUALITY_GATE = 0.50          # 拟合质量门限（低于此拒绝该帧估计）
# D1 修复：DFT 估计顶到扫描上限（RAMP_MAX_CYCLES+2=62）附近时不可信，
# 强制换快斜坡档复测（防止顶上限的泄漏值被 accept）。
RAMP_SUSPECT_CYCLES = 55
# D12 修复(2026-10-22)：N>=RAMP_CONFIRM_CYCLES 时即使拟合质量好也换快斜坡
# 复核一次。高频输入在 1kHz 斜坡下每周期像素数过少，
# 半周期 3px 处于欠采样边缘，单次估计分辨率不足会锁飘；换快斜坡把 N 压到
# <25（每周期 >=12px）再 accept。用户实测"只扫一次就换 9959、正弦太密集、
# 频率不稳定"正是此路径。
RAMP_CONFIRM_CYCLES = 20
# D14 修复(2026-10-22)：斜坡满幅高度改为 config 校准。旧实现假设 DAC
# 恰 3.3V@0.5V/div=6.6div；F407 DAC 实际输出常 3.2~3.3V（供电压降），
# 1% 高度误差在 10kHz 时就是 100Hz=整整 1 个网格档（实测估频系统性
# +1.4%：10.000kHz 被归成 101 档 10.100kHz，图形 100Hz 狂转 -> 旋转
# 修正 Δf 只有 Hz 级拉不回 -> 10s 超时 FAILED）。现场用示波器实测
# 斜坡 Vpp 后填 config ramp_full_height_div（= Vpp / 0.5V/div）。
RAMP_FULL_HEIGHT_DIV = 6.6
# 估频多帧中值（默认 7 帧）降帧间噪声，并配合 spread/MAD 一致性门限
# 拦截混叠别名；单纯增加中值帧数不能接受互相矛盾的周期估计。
# （2%），跨 100Hz 网格边界仍可能归错档。
RAMP_MEDIAN_FRAMES = 7
# A valid RAMP result must be repeatable across frames.  The old estimator
# accepted the median of mutually contradictory aliases (for example
# 1.7/10.0/0.8/9.6/3.4 cycles), which is how high-frequency inputs were
# incorrectly locked to a low-frequency DDS setting.
RAMP_MAX_RELATIVE_SPREAD = 0.12
RAMP_MAX_RELATIVE_MAD = 0.05
RAMP_UNSTABLE_SAW_HZ = 4000
RAMP_DISCARD_FRAMES_AFTER_APPLIED = 1
RAMP_GRID_ROUND = 100             # 估频归整到 100Hz 网格（用户明确步进 100Hz）
# v3.1（根因2修复）：局部网格纠错扫描（RAMP_LOCAL_SWEEP 落地）。错档
# ≥1 档（≥100Hz）时李萨如图形以 ≥50 转/s 旋转，30ms 曝光积分成实心
# 包络；正确档为静态细轨迹。用 fill_ratio 区分（与幅度/相位无关），
# 在 ±RAMP_LOCAL_SWEEP_RANGE_GRID 档内 V 型快查 + 粗扫 + ±1 细化拉回。
RAMP_LOCAL_SWEEP_RANGE_GRID = 12  # 局部网格扫描范围 ±12 档
RAMP_SWEEP_FILL_TRIGGER = 0.35    # fill 高于此 -> 图形旋转（错档嫌疑）
RAMP_SWEEP_STATIC_FILL = 0.30     # fill 低于此判定静态细轨迹（档位正确）
RAMP_SWEEP_FILL_MARGIN = 0.08     # V 型判据：中心档须比两侧低此值以上
RAMP_SWEEP_FRAMES = 2             # 每档采集帧数（fill 取中值）
RAMP_SWEEP_MAX_ATTEMPTS = 2       # 整个控制周期最多启动的扫描次数
RAMP_MIN_POINTS = 60              # 斜坡轨迹最少点数（行质心可用下限）

# ---------------------------------------------------------------------------
# 小型 MicroPython 安全辅助函数
# ---------------------------------------------------------------------------

def ticks_ms():
    try:
        return time.ticks_ms()
    except AttributeError:
        return int(time.monotonic() * 1000.0)

def ticks_diff(newer, older):
    try:
        return time.ticks_diff(int(newer), int(older))
    except AttributeError:
        return int(newer) - int(older)

def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value

def ramp_full_height_px(roi_height):
    """锯齿斜坡满幅（0..RAMP_AMP_MV）在 ROI 内占据的像素高度。

    斜坡只占 RAMP_AMP_MV/VOLTS_PER_DIV 个格（3.3V @0.5V/div = 6.6 格），
    不是 ROI 全高 GRID_DIVISIONS_Y（8 格）。estimate_ramp_cycles 的可见
    高度补偿必须以这个物理满幅高度为基准，否则 3.3V 斜坡会稳定高估
    N 达 8/6.6≈1.212 倍。
    D14：满幅高度由 RAMP_FULL_HEIGHT_DIV 决定（默认 6.6，现场按示波器
    实测斜坡 Vpp 校准，消除 DAC 电压偏差导致的系统性估频偏移）。
    """
    try:
        height = float(roi_height)
        full_div = float(RAMP_FULL_HEIGHT_DIV)
        pixels = full_div * (height / float(GRID_DIVISIONS_Y))
    except (TypeError, ValueError, ZeroDivisionError):
        return roi_height
    return clamp(pixels, 1.0, height)

def integer_gcd(left, right):
    left = abs(int(left))
    right = abs(int(right))
    while right:
        left, right = right, left % right
    return left

def median(values):
    if not values:
        return None
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) & 1:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])

def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = int((len(ordered) - 1) * float(fraction) + 0.5)
    index = int(clamp(index, 0, len(ordered) - 1))
    return ordered[index]

def wrap_phase_deg(value):
    return float(value) % 360.0

def wrap_phase_millideg(value):
    return int(value) % 360000

def phase_delta_deg(new_phase, old_phase):
    """从 old_phase 到 new_phase 的最短有符号相位差，范围 [-180, 180)。"""
    return (float(new_phase) - float(old_phase) + 180.0) % 360.0 - 180.0

def unwrap_phase_degrees(values):
    if not values:
        return []
    result = [float(values[0])]
    previous_wrapped = float(values[0])
    for value in values[1:]:
        wrapped = float(value)
        result.append(result[-1] + phase_delta_deg(wrapped, previous_wrapped))
        previous_wrapped = wrapped
    return result

def unwrap_axial_degrees(values):
    # PCA axes are unoriented lines, so their angle repeats every 180 degrees.
    if not values:
        return []
    result = [float(values[0])]
    for wrapped in values[1:]:
        delta = (float(wrapped) - result[-1] + 90.0) % 180.0 - 90.0
        result.append(result[-1] + delta)
    return result


def rotation_delta_hz(deg_per_sec):
    """主轴角速度(°/s) -> 频率差 Δf(Hz)。

    李萨如图形主轴以 Δf 旋转：1:1 图形转 180°/s 对应 Δf=1Hz（v2 标定口径，
    ROTATION_HZ_PER_TURN_DEG 为真机标定项；2:1 目标需重新标定）。
    符号约定：主轴正向旋转（角度递增）表示 fy > fx，Δf 为正。
    """
    return float(deg_per_sec) / ROTATION_HZ_PER_TURN_DEG

def reduced_ratio(count_x, count_y):
    count_x = int(count_x)
    count_y = int(count_y)
    if count_x <= 0 or count_y <= 0:
        return (0, 0)
    divisor = integer_gcd(count_x, count_y)
    return (count_x // divisor, count_y // divisor)

def strict_ratio_from_counts(left, right, top, bottom):
    """返回 (valid, reason, ratio_y, ratio_x)，不做计数修复。"""
    left = int(left)
    right = int(right)
    top = int(top)
    bottom = int(bottom)
    if left <= 0 or top <= 0:
        return (False, "NO_CONTACT", 0, 0)
    if left != right:
        return (False, "LEFT_RIGHT_MISMATCH", 0, 0)
    if top != bottom:
        return (False, "TOP_BOTTOM_MISMATCH", 0, 0)
    if left > MAX_TOUCH_COUNT or top > MAX_TOUCH_COUNT:
        return (False, "TOUCH_COUNT_OVERFLOW", 0, 0)
    ratio_x, ratio_y = reduced_ratio(left, top)
    return (True, None, ratio_y, ratio_x)

def amplitude_feedforward(frequency_grid, target_span_div):
    """对校准指令插值，并按请求的 span 缩放。"""
    frequency_grid = int(frequency_grid)
    if frequency_grid <= DDS_AMPLITUDE_TABLE[0][0]:
        reference_mv = float(DDS_AMPLITUDE_TABLE[0][1])
    elif frequency_grid >= DDS_AMPLITUDE_TABLE[-1][0]:
        reference_mv = float(DDS_AMPLITUDE_TABLE[-1][1])
    else:
        reference_mv = float(DDS_AMPLITUDE_TABLE[-1][1])
        for index in range(1, len(DDS_AMPLITUDE_TABLE)):
            low_grid, low_mv = DDS_AMPLITUDE_TABLE[index - 1]
            high_grid, high_mv = DDS_AMPLITUDE_TABLE[index]
            if frequency_grid <= high_grid:
                fraction = (
                    float(frequency_grid - low_grid)
                    / float(high_grid - low_grid)
                )
                reference_mv = low_mv + fraction * (high_mv - low_mv)
                break
    command = reference_mv * float(target_span_div) / DDS_REFERENCE_SPAN_DIV
    return int(clamp(round(command), DDS_AMPLITUDE_MIN_MV,
                     DDS_AMPLITUDE_MAX_MV))

# ---------------------------------------------------------------------------
# 真实的相位敏感形状评分
# ---------------------------------------------------------------------------

def _point_moments(points):
    if points is None or len(points) < 3:
        return None
    mean_x = sum(point[0] for point in points) / float(len(points))
    mean_y = sum(point[1] for point in points) / float(len(points))
    cxx = 0.0
    cyy = 0.0
    cxy = 0.0
    for x, y in points:
        dx = x - mean_x
        dy = y - mean_y
        cxx += dx * dx
        cyy += dy * dy
        cxy += dx * dy
    scale = float(len(points))
    cxx /= scale
    cyy /= scale
    cxy /= scale
    root = math.sqrt(max(0.0, (cxx - cyy) * (cxx - cyy)
                             + 4.0 * cxy * cxy))
    large = 0.5 * (cxx + cyy + root)
    small = max(0.0, 0.5 * (cxx + cyy - root))
    theta = 0.5 * math.atan2(2.0 * cxy, cxx - cyy)
    return (mean_x, mean_y, cxx, cyy, cxy, large, small, theta)

def diagonal_shape_metrics(points):
    """div 坐标下的 PCA/TLS 直线误差；两条斜率任一有效即可。"""
    moments = _point_moments(points)
    if moments is None:
        return {"error": 999.0, "score": 0.0, "pass": False}
    mean_x, mean_y, cxx, cyy, cxy, large, small, theta = moments
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)
    eigen_ratio = small / max(large, 1.0e-9)
    orthogonal_rms = math.sqrt(small)
    ux = math.cos(theta)
    uy = math.sin(theta)
    # 正确的 TLS 角点对由协方差符号选出。
    if cxy >= 0.0:
        corners = ((min(xs), min(ys)), (max(xs), max(ys)))
    else:
        corners = ((min(xs), max(ys)), (max(xs), min(ys)))
    corner_error = 0.0
    for corner_x, corner_y in corners:
        dx = corner_x - mean_x
        dy = corner_y - mean_y
        corner_error += abs(-uy * dx + ux * dy)
    corner_error *= 0.5
    span_error = abs(span_x - span_y)
    error = (
        4.0 * eigen_ratio + 2.0 * orthogonal_rms
        + 0.5 * corner_error + 0.5 * span_error
    )
    passed = bool(
        eigen_ratio <= 0.03 and orthogonal_rms <= 0.12
        and span_error <= 0.12
    )
    return {
        "error": error,
        "score": 1.0 / (1.0 + error),
        "pass": passed,
        "eigen_ratio": eigen_ratio,
        "orthogonal_rms_div": orthogonal_rms,
        "corner_error_div": corner_error,
    }

def circle_shape_metrics(points, scope_center=(0.0, 0.0)):
    """实际归一化径向变异系数，加上相关性/跨度/中心误差。"""
    moments = _point_moments(points)
    if moments is None:
        return {"error": 999.0, "score": 0.0, "pass": False,
                "radial_cv": 999.0}
    mean_x, mean_y, cxx, cyy, cxy, large, small, theta = moments
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    low_x = percentile(xs, 0.01)
    high_x = percentile(xs, 0.99)
    low_y = percentile(ys, 0.01)
    high_y = percentile(ys, 0.99)
    span_x = max(1.0e-6, high_x - low_x)
    span_y = max(1.0e-6, high_y - low_y)
    center_x = 0.5 * (low_x + high_x)
    center_y = 0.5 * (low_y + high_y)
    radii = []
    for x, y in points:
        nx = (x - center_x) / (0.5 * span_x)
        ny = (y - center_y) / (0.5 * span_y)
        radii.append(math.sqrt(nx * nx + ny * ny))
    radius_mean = sum(radii) / max(1, len(radii))
    radius_variance = 0.0
    for radius in radii:
        radius_variance += (radius - radius_mean) * (radius - radius_mean)
    radius_variance /= max(1, len(radii))
    radial_cv = math.sqrt(radius_variance) / max(radius_mean, 1.0e-9)
    correlation = cxy / math.sqrt(max(cxx * cyy, 1.0e-12))
    span_difference = abs(span_x - span_y)
    span_relative = span_difference / max(span_x, 1.0e-9)
    center_offset = math.sqrt(
        (center_x - scope_center[0]) ** 2
        + (center_y - scope_center[1]) ** 2
    )
    error = (
        2.0 * abs(correlation) + 2.0 * span_relative
        + 4.0 * radial_cv + center_offset
    )
    passed = bool(
        abs(correlation) <= 0.08 and span_difference <= 0.12
        and radial_cv <= 0.06 and center_offset <= 0.15
    )
    return {
        "error": error,
        "score": 1.0 / (1.0 + error),
        "pass": passed,
        "correlation": correlation,
        "radial_cv": radial_cv,
        "span_difference_div": span_difference,
        "center_offset_div": center_offset,
    }

def _expanded_cells(cells, bins):
    expanded = set()
    for cell in cells:
        x = cell % bins
        y = cell // bins
        for dy in (-1, 0, 1):
            ny = y + dy
            if ny < 0 or ny >= bins:
                continue
            for dx in (-1, 0, 1):
                nx = x + dx
                if 0 <= nx < bins:
                    expanded.add(ny * bins + nx)
    return expanded

def _set_iou(first, second):
    if not first and not second:
        return 1.0
    union = first | second
    if not union:
        return 0.0
    return len(first & second) / float(len(union))

def infinity_shape_metrics(points, bins=64, scope_center=(0.0, 0.0)):
    """占用镜像对称、中心穿越与瓣平衡评分。"""
    if points is None or len(points) < 20:
        return {"error": 999.0, "score": 0.0, "pass": False}
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    low_x = percentile(xs, 0.01)
    high_x = percentile(xs, 0.99)
    low_y = percentile(ys, 0.01)
    high_y = percentile(ys, 0.99)
    span_x = max(1.0e-6, high_x - low_x)
    span_y = max(1.0e-6, high_y - low_y)
    center_x = 0.5 * (low_x + high_x)
    center_y = 0.5 * (low_y + high_y)
    cells = set()
    left_points = 0
    right_points = 0
    center_cross_distance = 999.0
    for x, y in points:
        bx = int(clamp(round((x - low_x) * (bins - 1) / span_x),
                       0, bins - 1))
        by = int(clamp(round((y - low_y) * (bins - 1) / span_y),
                       0, bins - 1))
        cells.add(by * bins + bx)
        if x < center_x:
            left_points += 1
        elif x > center_x:
            right_points += 1
        distance = math.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
        if distance < center_cross_distance:
            center_cross_distance = distance
    cells = _expanded_cells(cells, bins)
    mirror_lr = set()
    mirror_tb = set()
    for cell in cells:
        x = cell % bins
        y = cell // bins
        mirror_lr.add(y * bins + (bins - 1 - x))
        mirror_tb.add((bins - 1 - y) * bins + x)
    iou_lr = _set_iou(cells, mirror_lr)
    iou_tb = _set_iou(cells, mirror_tb)
    lobe_balance = abs(left_points - right_points) / float(
        max(1, left_points + right_points)
    )
    center_offset = math.sqrt(
        (center_x - scope_center[0]) ** 2
        + (center_y - scope_center[1]) ** 2
    )
    span_difference = abs(span_x - span_y)
    error = (
        3.0 * center_cross_distance
        + 2.0 * (1.0 - iou_lr)
        + 2.0 * (1.0 - iou_tb)
        + 2.0 * lobe_balance + center_offset
    )
    passed = bool(
        center_cross_distance <= 0.15
        and iou_lr >= 0.90 and iou_tb >= 0.90
        and lobe_balance <= 0.08 and span_difference <= 0.12
        and center_offset <= 0.15
    )
    return {
        "error": error,
        "score": 1.0 / (1.0 + error),
        "pass": passed,
        "center_cross_distance_div": center_cross_distance,
        "mirror_iou_lr": iou_lr,
        "mirror_iou_tb": iou_tb,
        "lobe_area_imbalance": lobe_balance,
        "center_offset_div": center_offset,
        "span_difference_div": span_difference,
    }

# ---------------------------------------------------------------------------
# 黄色轨迹提取与严格四边多尺度计数
# ---------------------------------------------------------------------------

def pixel_is_white(pixel):
    """接受 CanMV 二值像素，兼容整数、RGB565 或 RGB 元组形式。"""
    if isinstance(pixel, tuple):
        if len(pixel) >= 3:
            return pixel[0] + pixel[1] + pixel[2] > 300
        return bool(pixel[0])
    return pixel != 0

def collect_yellow_points(img, roi=GRID_RECT, step=None):
    """采集黄色轨迹像素，只在 blob 包围盒内扫描。

    旧实现把整个 ROI 二值化后逐像素遍历（2px 步长下每帧约 49k 次 get_pixel），
    占满帧预算。find_blobs() 已在 C 层返回轨迹包围盒，因此把 Python 像素遍历
    限制在该盒加小边距内。返回的点集含义相同（轨迹像素都在盒内），
    下游投影/形状消费者无需改动。
    """
    # 背景差分 AND 黄色：差分启用且背景就绪时改走差分检测，
    # 排除静止的黄色光标/网格/UI，只保留黄色且相对空背景有变化的运动轨迹。
    global MASK_ACCESS_MODE
    global BG_WARNED
    if BG_ENABLED and BACKGROUND is not None and BG_ROI is not None and close_roi(BG_ROI, roi):
        return collect_diff_points(img, roi)
    if BG_ENABLED and not BG_WARNED:
        BG_WARNED = True
        print('[背景] 警告：ROI=%s 无有效背景图，回退纯颜色提取' % (str(roi),))
    roi_x, roi_y, roi_w, roi_h = roi
    try:
        blobs = img.find_blobs(
            [YELLOW_THRESHOLD], roi=roi,
            pixels_threshold=FAST_BLOB_PIXELS_THRESHOLD, merge=True
        )
    except Exception as error:
        print("[提取] find_blobs 异常: %s" % error)
        return []
    if not blobs:
        return []
    # 轨迹线是屏幕上面积最大的黄色连通域；菜单/刻度文字等杂散 UI
    # 是独立小块。只保留像素最多的 blob（轨迹候选），排除文字干扰。
    best = None
    best_pixels = 0
    for blob in blobs:
        try:
            p = int(blob.pixels())
        except Exception:
            continue
        if p > best_pixels:
            best_pixels = p
            best = blob
    if best is None:
        return []
    # 以最大 blob 的包围盒为扫描范围（坐标相对全图）
    min_x = roi_x + roi_w
    min_y = roi_y + roi_h
    max_x = roi_x
    max_y = roi_y
    for blob in (best,):
        try:
            x, y, w, h = blob.rect()
        except Exception:
            continue
        if x < min_x:
            min_x = x
        if y < min_y:
            min_y = y
        if x + w > max_x:
            max_x = x + w
        if y + h > max_y:
            max_y = y + h
    # 与 ROI 求交，并向外扩 4 px 容忍边界
    scan_x0 = max(roi_x, min_x - 4)
    scan_y0 = max(roi_y, min_y - 4)
    scan_x1 = min(roi_x + roi_w, max_x + 4)
    scan_y1 = min(roi_y + roi_h, max_y + 4)
    if scan_x1 <= scan_x0 or scan_y1 <= scan_y0:
        return []
    # FPS-OPT: 采样步长可由调用方指定。counts/span 模式（SEARCH/MEASURE_X）
    # 只需要四边接触数与跨度，粗采样即可（实测满格包络约 8000 采样点时
    # 四边投影耗时 120ms+，是 SEARCH 阶段 10s 预算爆掉的直接原因）。
    # full 模式（PHASE/AMPLITUDE/LOCK/HOLD）保持细采样保形状评分精度。
    if step is None:
        step = TRACE_SAMPLE_STEP
    points = []
    try:
        mask = img.copy(roi=roi)
        mask.binary([YELLOW_THRESHOLD])
        # FPS-OPT: 用 memoryview 直接读 RGB565 掩码缓冲区，替代每像素
        # mask.get_pixel() 调用（Python 函数调用开销远大于内存索引）。
        # 与 collect_diff_points 中 binary_mask_buffer 同一套加速路径。
        mask_view, mask_stride = binary_mask_buffer(mask, roi_w, roi_h) if MASK_BUFFER_ENABLED else (None, 0)
        if mask_view is not None:
            MASK_ACCESS_MODE = 'buffer'
        else:
            MASK_ACCESS_MODE = 'get_pixel'
        mask_get = mask.get_pixel
        for y in range(scan_y0, scan_y1, step):
            mask_row = (y - roi_y) * roi_w
            for x in range(scan_x0, scan_x1, step):
                # mask 是 roi 的拷贝，坐标相对 ROI 原点
                if mask_view is not None:
                    if not mask_view[(mask_row + (x - roi_x)) * mask_stride]:
                        continue
                elif not pixel_is_white(mask_get(x - roi_x, y - roi_y)):
                    continue
                points.append((x - roi_x, y - roi_y))
    except Exception as error:
        print("[提取] 黄色掩码异常: %s" % error)
        points = []
    mask_view = None
    mask = None
    return points

def fast_trace_stats(img, roi=GRID_RECT):
    """一次 C 层扫描，返回轨迹面积/包围盒/填充率。

    这是 SEARCH 阶段观测器：完全避开 Python 像素遍历与四边投影。
    ``fill_ratio`` 即搜索预过滤器用的静态门限。无轨迹可见时返回 None。
    """
    roi_x, roi_y, roi_w, roi_h = roi
    try:
        blobs = img.find_blobs(
            [YELLOW_THRESHOLD], roi=roi,
            pixels_threshold=FAST_BLOB_PIXELS_THRESHOLD, merge=True
        )
    except Exception as error:
        print("[快测] find_blobs 异常: %s" % error)
        return None
    total = 0
    min_x = roi_x + roi_w
    min_y = roi_y + roi_h
    max_x = roi_x
    max_y = roi_y
    found = False
    best_pixels = 0
    best_rect = None
    for blob in blobs:
        pixels = int(blob.pixels())
        if pixels > best_pixels:
            best_pixels = pixels
            try:
                best_rect = blob.rect()
            except Exception:
                best_rect = None
    if best_rect is not None:
        x, y, w, h = best_rect
        found = True
        min_x, min_y, max_x, max_y = x, y, x + w, y + h
        total = best_pixels
    if not found or total < FAST_BLOB_PIXELS_THRESHOLD:
        return None
    bbox_w = max(1, max_x - min_x)
    bbox_h = max(1, max_y - min_y)
    fill = total / float(bbox_w * bbox_h)
    return {
        "valid": True,
        "pixels": int(total),
        "bbox": (int(min_x), int(min_y), int(max_x), int(max_y)),
        "fill_ratio": fill,
    }

def make_fast_measurement(stats, roi=GRID_RECT):
    """由 fast_trace_stats() 构造轻量测量字典。"""
    if stats is None:
        return empty_measurement("NO_TRACE")
    roi_x, roi_y, roi_w, roi_h = roi
    min_x, min_y, max_x, max_y = stats["bbox"]
    px_per_div_x = float(roi_w) / GRID_DIVISIONS_X
    px_per_div_y = float(roi_h) / GRID_DIVISIONS_Y
    result = empty_measurement()
    result.update({
        "valid": True,
        "invalid_reason": None,
        "fill_ratio": float(stats["fill_ratio"]),
        "trace_pixels": int(stats["pixels"]),
        "span_x_div": (max_x - min_x) / px_per_div_x,
        "span_y_div": (max_y - min_y) / px_per_div_y,
        "center_x_div": (
            0.5 * (min_x + max_x) - 0.5 * (roi_x + roi_w - 1)
        ) / px_per_div_x,
        "center_y_div": (
            0.5 * (roi_y + roi_h - 1) - 0.5 * (min_y + max_y)
        ) / px_per_div_y,
        "bounds_div": (
            (min_x - 0.5 * (roi_x + roi_w - 1)) / px_per_div_x,
            (max_x - 0.5 * (roi_x + roi_w - 1)) / px_per_div_x,
            (0.5 * (roi_y + roi_h - 1) - max_y) / px_per_div_y,
            (0.5 * (roi_y + roi_h - 1) - min_y) / px_per_div_y,
        ),
    })
    return result

def robust_trace_bounds(points, step=TRACE_SAMPLE_STEP):
    # D5 修复：点数门限用等效满采样口径（同 analyze_trace_points），
    # 否则 counts 模式 step=6 时稀疏点集(如 113 点圆)在这里二次误判。
    if points is None or len(points) * (
            float(step) / TRACE_SAMPLE_STEP) ** 2 < MIN_TRACE_POINTS:
        return None
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    low_x = percentile(xs, TRIM_FRACTION)
    high_x = percentile(xs, 1.0 - TRIM_FRACTION)
    low_y = percentile(ys, TRIM_FRACTION)
    high_y = percentile(ys, 1.0 - TRIM_FRACTION)
    if high_x <= low_x or high_y < low_y:
        return None
    return (int(low_x), int(high_x), int(low_y), int(high_y))

def trace_edge_cut_div(points, roi_width, roi_height):
    """四条边带内轨迹点沿边分布的最大跨度（div）。

    过幅圆/∞ 形的可见部分是从屏幕四边穿出的圆弧，边带内的点沿边
    跨度可达数 div；恰满幅时图形与边缘相切（或对角线过角点），
    边带点沿边跨度约 1.5 div。返回 0.0 表示未检测到边缘点。
    """
    if not points:
        return 0.0
    width = float(roi_width)
    height = float(roi_height)
    left = []
    right = []
    top = []
    bottom = []
    for x, y in points:
        if x <= EDGE_CUT_BAND_PX:
            left.append(y)
        if x >= width - 1.0 - EDGE_CUT_BAND_PX:
            right.append(y)
        if y <= EDGE_CUT_BAND_PX:
            top.append(x)
        if y >= height - 1.0 - EDGE_CUT_BAND_PX:
            bottom.append(x)
    best = 0.0
    for values, axis_px, divisions in (
        (left, height, GRID_DIVISIONS_Y),
        (right, height, GRID_DIVISIONS_Y),
        (top, width, GRID_DIVISIONS_X),
        (bottom, width, GRID_DIVISIONS_X),
    ):
        if len(values) < 2:
            continue
        spread = (max(values) - min(values)) * divisions / max(
            1.0, axis_px
        )
        if spread > best:
            best = spread
    return best

def _median_filter(values, radius=1):
    result = [0.0] * len(values)
    for index in range(len(values)):
        low = max(0, index - radius)
        high = min(len(values), index + radius + 1)
        result[index] = float(median(values[low:high]))
    return result

def _smooth_projection(values):
    filtered = _median_filter(values, 1)
    result = [0.0] * len(filtered)
    for index in range(len(filtered)):
        low = max(0, index - 2)
        high = min(len(filtered), index + 3)
        result[index] = sum(filtered[low:high]) / float(high - low)
    return result

def projection_peak_positions(projection, min_distance=PEAK_MIN_DISTANCE_PX):
    """每个边界触点找出一条显著的连续段。

    粗示波器轨迹常在一个真实切点附近产生两个邻近局部极大值，按局部极大值计数
    会把理想圆拆成多个假触点。阈值化连续段在保留显著性门限的同时，
    合并同一切点内的亚像素/采样间隙。
    """
    if projection is None or len(projection) < 3:
        return []
    values = _smooth_projection(projection)
    positive = [value for value in values if value > 0.0]
    if not positive:
        return []
    p95 = percentile(positive, 0.95)
    threshold = max(0.25, 0.12 * float(p95))
    merge_gap = max(2, int(TRACE_SAMPLE_STEP) + 2)
    runs = []
    start = None
    previous = None
    for index in range(len(values)):
        if values[index] < threshold:
            continue
        if start is None:
            start = index
        elif index - previous > merge_gap:
            runs.append((start, previous))
            start = index
        previous = index
    if start is not None:
        runs.append((start, previous))

    candidates = []
    for start, end in runs:
        weight = 0.0
        weighted_position = 0.0
        height = 0.0
        for index in range(start, end + 1):
            value = values[index]
            weight += value
            weighted_position += index * value
            height = max(height, value)
        if weight <= 0.0:
            continue
        position = int(weighted_position / weight + 0.5)
        candidates.append((height, position))

    # 光学分辨率内的触点刻意合并；随后的双相位与第二探针确认可防止误判频率。
    candidates.sort(reverse=True)
    accepted = []
    for height, position in candidates:
        separated = True
        for other in accepted:
            if abs(position - other) < int(min_distance):
                separated = False
                break
        if separated:
            accepted.append(position)
    accepted.sort()
    return accepted

def _projection_for_side(points, bounds, side, band_ratio):
    low_x, high_x, low_y, high_y = bounds
    span_x = max(1, high_x - low_x + 1)
    span_y = max(1, high_y - low_y + 1)
    if side in ("left", "right"):
        projection = [0] * (high_y + 1)
        band = max(3, int(round(span_x * float(band_ratio))))
        if side == "left":
            limit_low = low_x
            limit_high = low_x + band
        else:
            limit_low = high_x - band
            limit_high = high_x
        for x, y in points:
            if limit_low <= x <= limit_high and low_y <= y <= high_y:
                projection[y] += 1
        return projection
    projection = [0] * (high_x + 1)
    band = max(3, int(round(span_y * float(band_ratio))))
    if side == "top":
        limit_low = low_y
        limit_high = low_y + band
    else:
        limit_low = high_y - band
        limit_high = high_y
    for x, y in points:
        if limit_low <= y <= limit_high and low_x <= x <= high_x:
            projection[x] += 1
    return projection

def agreed_contact_positions(counts, positions_by_scale, count):
    """要求触点位置一致，而不只是峰值数量相同。"""
    agreeing = [positions_by_scale[i] for i in range(len(counts)) if counts[i] == count]
    if count <= 0 or len(agreeing) < MULTISCALE_AGREE_COUNT:
        return None
    groups = [agreeing]
    if len(agreeing) > MULTISCALE_AGREE_COUNT:
        groups += [agreeing[:i]+agreeing[i+1:] for i in range(len(agreeing))]
    best = None
    best_size = 0
    best_spread = None
    for group in groups:
        spread = max(max(p[k] for p in group)-min(p[k] for p in group)
                     for k in range(count))
        if spread > CONTACT_POSITION_TOLERANCE_PX:
            continue
        if len(group) > best_size or (len(group) == best_size and spread < best_spread):
            best = [int(round(median([p[k] for p in group]))) for k in range(count)]
            best_size = len(group)
            best_spread = spread
    return best

def snap_contact_points(edge_points, bounds, side, positions):
    """把标记放在每个一致峰值附近真实的前景点上。"""
    low_x, high_x, low_y, high_y = bounds
    contacts = []
    for position in positions:
        best = None
        best_distance = None
        for x,y in edge_points:
            if not (low_x<=x<=high_x and low_y<=y<=high_y):
                continue
            tangent = y if side in ("left", "right") else x
            if abs(tangent-position) > CONTACT_POSITION_TOLERANCE_PX:
                continue
            if side == "left": normal = x-low_x
            elif side == "right": normal = high_x-x
            elif side == "top": normal = y-low_y
            else: normal = high_y-y
            distance = normal*normal+(tangent-position)*(tangent-position)
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best = (x,y)
        if best is None:
            return None
        contacts.append(best)
    return contacts

def multiscale_side_count(points, bounds, side):
    counts = []
    positions_by_scale = []
    low_x, high_x, low_y, high_y = bounds
    span = high_x-low_x+1 if side in ("left", "right") else high_y-low_y+1
    max_band = max(max(3,int(round(span*r))) for r in EDGE_BAND_RATIOS)
    # 大部分轨迹像素远离给定边。先过滤一次，避免为四种带宽各遍历一遍整条轨迹。
    if side == "left":
        edge_points = [(x,y) for x,y in points if low_x<=x<=low_x+max_band]
    elif side == "right":
        edge_points = [(x,y) for x,y in points if high_x-max_band<=x<=high_x]
    elif side == "top":
        edge_points = [(x,y) for x,y in points if low_y<=y<=low_y+max_band]
    else:
        edge_points = [(x,y) for x,y in points if high_y-max_band<=y<=high_y]
    band_cache = {}
    for ratio in EDGE_BAND_RATIOS:
        band = max(3,int(round(span*ratio)))
        if band not in band_cache:
            projection = _projection_for_side(edge_points, bounds, side, ratio)
            band_cache[band] = projection_peak_positions(projection)
        positions = band_cache[band]
        counts.append(len(positions))
        positions_by_scale.append(positions)
    best_count = 0
    best_votes = 0
    for count in counts:
        votes = counts.count(count)
        if votes > best_votes or (votes == best_votes and count < best_count):
            best_count = count
            best_votes = votes
    valid = bool(
        best_votes >= MULTISCALE_AGREE_COUNT
        and 0 < best_count <= MAX_TOUCH_COUNT
    )
    reason = None if valid else "PEAK_MERGED"
    contact_positions = []
    agreed_positions = None
    if valid:
        agreed_positions = agreed_contact_positions(counts, positions_by_scale, best_count)
        if agreed_positions is not None:
            contact_positions = snap_contact_points(edge_points, bounds, side, agreed_positions)
        if agreed_positions is None or contact_positions is None:
            valid = False
            contact_positions = []
            reason = "CONTACT_POSITION_UNSTABLE"
    return {
        "valid": valid,
        "count": best_count if valid else 0,
        "scale_counts": tuple(counts),
        "positions": tuple(positions_by_scale),
        "contact_positions": tuple(contact_positions),
        "agreed_positions": tuple(agreed_positions) if agreed_positions is not None else (),
        "reason": reason,
    }

def points_to_divisions(points, roi_width, roi_height):
    pixels_per_div_x = float(roi_width) / GRID_DIVISIONS_X
    pixels_per_div_y = float(roi_height) / GRID_DIVISIONS_Y
    center_x = 0.5 * (roi_width - 1)
    center_y = 0.5 * (roi_height - 1)
    converted = []
    for x, y in points:
        converted.append((
            (x - center_x) / pixels_per_div_x,
            (center_y - y) / pixels_per_div_y,
        ))
    return converted

def empty_measurement(reason="NO_TRACE"):
    return {
        "valid": False,
        "strict_pair_valid": False,
        "invalid_reason": reason,
        "left": 0,
        "right": 0,
        "top": 0,
        "bottom": 0,
        "ratio_x": 0,
        "ratio_y": 0,
        "trace_pixels": 0,
        "span_x_div": 0.0,
        "span_y_div": 0.0,
        "center_x_div": 0.0,
        "center_y_div": 0.0,
        "bounds_div": None,
        "score_diagonal": 0.0,
        "score_circle": 0.0,
        "score_infinity": 0.0,
        "error_diagonal": 999.0,
        "error_circle": 999.0,
        "error_infinity": 999.0,
        "diagonal_pass": False,
        "circle_pass": False,
        "infinity_pass": False,
        "shape": "unknown",
        "shape_confidence": 0.0,
        "principal_axis_angle_deg": None,
    }

# ==================================================================
# v3：固定周期短斜坡图像分析（行质心 + 整数周期 DFT + LSQ 正弦拟合）
# ==================================================================

def ramp_row_centroids(points, roi):
    """把斜坡展开轨迹按行（Y=斜坡相位轴）投影为行质心 X。

    返回 (rows, xs)：rows 为 ROI 局部 Y 坐标（斜坡相位），xs 为该行
    黄色轨迹的质心 X（输入正弦在对应相位的取值）。每行取均值抗噪，
    单点行也计入（细轨迹每行只有一个点）。整条轨迹是 x=x(y) 的单值
    曲线，行质心即曲线采样。v3.2：F407 休止电平改到量程中点后，停住带
    那一行是满宽横线（像素数远高于普通行），按像素数整段剔除。
    """
    rx, ry, rw, rh = roi
    if not points or rh <= 0:
        return [], []
    rows = {}
    for lx, ly in points:
        bucket = rows.get(ly)
        if bucket is None:
            rows[ly] = [lx, 1]
        else:
            bucket[0] += lx
            bucket[1] += 1
    order = sorted(rows.items())
    # v3.2（配合 F407 休止电平 0V->量程中点）：休止的 9ms 光点在屏中部画
    # 一条满宽横线，该行像素数是普通行的几十倍，质心被拉到屏中心而非真实
    # 正弦值，会把中段拟合带歪。用行像素数的稳健上界整段识别剔除；若剔除
    # 后行数不足 60%（误判），则保留全部，避免把有效数据整段丢掉。
    if len(order) >= 24:
        counts = sorted(c for _, (_, c) in order)
        med = counts[len(counts) // 2]
        cut = max(8, 4 * med)
        kept = [(row, b) for row, b in order if b[1] < cut]
        if len(kept) >= max(12, (len(order) * 3) // 5):
            order = kept
    rows_out = []
    xs = []
    for row, (sum_x, count) in order:
        rows_out.append(row)
        xs.append(sum_x / float(count))
    return rows_out, xs


def _ramp_dft_power(xs, rows=None):
    """整数周期 DFT 粗估：对 1..min(n/2, RAMP_MAX_CYCLES+2) 扫周期数。

    返回 (best_n, best_power)。纯 Python 内积（MicroPython 无 numpy fft），
    n≈80~380 点时单次全扫约几千次乘加，毫秒级可接受。

    D6 修复(2026-10-22)：相位用实际行坐标归一化 u=(row-row0)/row_span，
    与 _ramp_fit_error 同一口径。旧实现用枚举索引当均匀相位，轨迹断行/
    回扫残留污染时 DFT 与 LSQ 互相矛盾，黄金分割会收敛到错误 N。
    rows 缺失或长度不匹配时回退均匀假设（保持旧调用兼容）。
    """
    n = len(xs)
    if n < 8:
        return 0, 0.0
    mean_x = sum(xs) / float(n)
    ac = [v - mean_x for v in xs]
    us = None
    if rows is not None and len(rows) == n:
        row0 = float(rows[0])
        row_span = float(rows[-1]) - row0
        if row_span > 0.0:
            us = [(r - row0) / row_span for r in rows]
    if us is None:
        us = [i / float(n) for i in range(n)]
    best_n = 0
    best_power = 0.0
    max_cycles = min(n // 2, RAMP_MAX_CYCLES + 2)
    two_pi = 6.283185307179586
    for k in range(1, max_cycles + 1):
        re = 0.0
        im = 0.0
        for i, v in enumerate(ac):
            phase = two_pi * k * us[i]
            re += v * math.cos(phase)
            im += v * math.sin(phase)
        power = re * re + im * im
        if power > best_power:
            best_power = power
            best_n = k
    return best_n, best_power


def _solve4x4(a, b):
    """解 4 元线性方程组 Ax=b（高斯消元，列主元）。奇异返回 None。"""
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    size = 4
    for col in range(size):
        pivot = col
        best = abs(m[col][col])
        for r in range(col + 1, size):
            value = abs(m[r][col])
            if value > best:
                best = value
                pivot = r
        if best < 1e-12:
            return None
        if pivot != col:
            m[col], m[pivot] = m[pivot], m[col]
        pv = m[col][col]
        for c in range(col, size + 1):
            m[col][c] /= pv
        for r in range(size):
            if r == col or m[r][col] == 0.0:
                continue
            factor = m[r][col]
            for c in range(col, size + 1):
                m[r][c] -= factor * m[col][c]
    return [m[i][size] for i in range(size)]


def _ramp_fit_error(xs, rows, n_cycles, u_base, u_span):
    """给定周期数 n_cycles，最小二乘拟合 [sin, cos, 直流, 线性] 四项。

    rows 归一化为相位 u∈[0,1)：u=(row-u_base)/u_span，其中 u_base 为轨迹
    起始行（稳健下分位）、u_span 为轨迹可见高度（稳健上分位-下分位）。
    这里拟合的是"可见窗口内的周期数 N_vis"；真实斜坡窗口（完整锯齿
    周期）内的周期数由调用方按 N_true = N_vis·rh/u_span 补偿。
    线性项吸收斜坡非线/亮度漂移，直流项吸收垂直偏移。
    """
    m = len(xs)
    if m < 8:
        return 1e9
    span_u = float(u_span)
    if span_u <= 0.0:
        return 1e9
    base = float(u_base)
    u = [(r - base) / span_u for r in rows]
    two_pi_n = 6.283185307179586 * float(n_cycles)
    a11 = a12 = a13 = a14 = 0.0
    a22 = a23 = a24 = 0.0
    a33 = a34 = 0.0
    a44 = 0.0
    b1 = b2 = b3 = b4 = 0.0
    for i in range(m):
        s = math.sin(two_pi_n * u[i])
        c = math.cos(two_pi_n * u[i])
        x = u[i]
        y = xs[i]
        a11 += s * s; a12 += s * c; a13 += s; a14 += s * x
        a22 += c * c; a23 += c; a24 += c * x
        a33 += 1.0; a34 += x
        a44 += x * x
        b1 += y * s; b2 += y * c; b3 += y; b4 += y * x
    coeff = _solve4x4(
        [
            [a11, a12, a13, a14],
            [a12, a22, a23, a24],
            [a13, a23, a33, a34],
            [a14, a24, a34, a44],
        ],
        [b1, b2, b3, b4],
    )
    if coeff is None:
        return 1e9
    sse = 0.0
    for i in range(m):
        s = math.sin(two_pi_n * u[i])
        c = math.cos(two_pi_n * u[i])
        x = u[i]
        pred = coeff[0] * s + coeff[1] * c + coeff[2] + coeff[3] * x
        d = xs[i] - pred
        sse += d * d
    return sse / float(m)


def estimate_ramp_cycles(points, roi):
    """从斜坡展开轨迹估计输入频率对应的周期数 N=fx·T_ramp。

    流程：行质心 -> 整数周期 DFT 粗估（±1 周期窗口）-> 黄金分割
    扫 N 使 4 元 LSQ 拟合残差最小。返回 (n_cycles, quality)；
    quality∈[0,1] 为拟合质量（1-残差均方根/信号半摆幅）。
    轨迹点数/行数不足返回 (None, 0.0)。

    【可见高度补偿（关键）】DFT/拟合在 5%~95% 稳健区间内数出的是该区间
    的周期数 N_vis，归一化高度用的是 h_vis=(p95-p05)。完整斜坡窗口的
    物理高度 H_ramp 用轨迹自身 0.5%~99.5% 分位跨距自标定（≈0.99×H，
    除以 0.99 还原）：N_true = N_vis × (h_full/0.99) / h_vis。
    旧实现用 rh×RAMP_FULL_HEIGHT_DIV/8 网格标定高度，把 GRID_RECT 标定、
    DAC 实际 Vpp、示波器增益误差全部乘进 N（666 十组实测 +2.4~2.5%
    系统性高估，fx>2kHz 全部落错 100Hz 档，v3.1 根因1修复）。
    """
    rx, ry, rw, rh = roi
    if points is None or len(points) < RAMP_MIN_POINTS:
        return None, 0.0
    rows, xs = ramp_row_centroids(points, roi)
    if len(rows) < 12:
        return None, 0.0
    # 稳健可见范围（5%~95% 分位）：排除顶部/底部回扫线与离群行污染。
    p_lo = percentile(rows, 0.05)
    p_hi = percentile(rows, 0.95)
    # v3.1（根因1修复）：完整斜坡高度改用轨迹自身跨距自标定。
    # 0.5%~99.5% 分位跨距覆盖全部有效行（含 5%~95% 窗口外的轨迹），
    # 在未过滤的完整行序列上取值（rows 随后会被过滤重赋值）。
    p_lo_full = percentile(rows, 0.005)
    p_hi_full = percentile(rows, 0.995)
    h_vis = p_hi - p_lo
    h_full = p_hi_full - p_lo_full
    if h_vis < 0.1 * rh:
        return None, 0.0
    kept = [
        (r, x) for r, x in zip(rows, xs) if p_lo <= r <= p_hi
    ]
    if len(kept) < 12:
        return None, 0.0
    rows = [r for r, _ in kept]
    xs = [x for _, x in kept]
    n_dft, _ = _ramp_dft_power(xs, rows)   # D6: 传实际行坐标，断行不破坏相位
    if n_dft <= 0:
        return None, 0.0
    lo = max(0.5, float(n_dft - 1))
    hi = float(n_dft + 1)
    for _ in range(24):
        m1 = lo + (hi - lo) * 0.382
        m2 = lo + (hi - lo) * 0.618
        if _ramp_fit_error(xs, rows, m1, p_lo, h_vis) < \
                _ramp_fit_error(xs, rows, m2, p_lo, h_vis):
            hi = m2
        else:
            lo = m1
    n_vis = (lo + hi) * 0.5
    e_best = _ramp_fit_error(xs, rows, n_vis, p_lo, h_vis)
    span = max(xs) - min(xs)
    if span <= 0.0:
        return None, 0.0
    rms = math.sqrt(max(0.0, e_best))
    quality = clamp(1.0 - rms / (span * 0.5), 0.0, 1.0)
    # v3.1（根因1修复）：完整斜坡窗口内的周期数补偿改用轨迹自身跨距。
    # 旧实现 n_full = n_vis * ramp_full_height_px(rh) / h_vis 用网格标定
    # 高度（rh×RAMP_FULL_HEIGHT_DIV/8），把 GRID_RECT 标定、DAC 实际 Vpp
    # （供电压降常 3.2~3.3V）、示波器增益误差全部按同比例乘进 N——666
    # 十组实测稳定 +2.4~2.5% 系统性高估（1k→1.02 … 10k→10.26），fx>2kHz
    # 全部落错 100Hz 档。行分布均匀（斜坡线性扫过所有行 + 回扫竖线覆盖
    # 全部行），0.5%~99.5% 分位跨距 h_full ≈ 0.99×H（H 为完整斜坡高度），
    # 除以 0.99 还原 H 后与 h_vis 相比即纯几何补偿，所有标定常数相消：
    #     n_full = n_vis × (h_full/0.99) / h_vis
    # 休止线在量程中点（v3.2 为配合示波器 AC 耦合把休止电平从 0V 抬到
    # 量程中点），已由 ramp_row_centroids 按行像素数整段剔除；回扫竖线在
    # 固定 X 列（LSQ 直流项吸收），均不破坏 0.5% 分位窗口。跨距异常时退回旧
    # 网格标定口径兜底。
    if h_full > h_vis and h_full > 0.0:
        n_full = n_vis * (h_full / 0.99) / h_vis
    else:
        n_full = n_vis * ramp_full_height_px(rh) / h_vis
    return n_full, quality

def analyze_trace_points(points, roi_width=None, roi_height=None,
                         mode="full", target_shape=None, step=TRACE_SAMPLE_STEP):
    """CanMV 帧与 CPython 测试共用的纯轨迹分析。

    D5 修复(2026-10-22)：新增 step 参数。旧实现固定按 TRACE_SAMPLE_STEP=3
    折算 trace_pixels，counts/span 模式实际步长 6 时 fill_ratio 低估 4 倍，
    且采样点数少 4 倍导致小图形误判 NO_TRACE。
    """
    if roi_width is None:
        roi_width = GRID_RECT[2]
    if roi_height is None:
        roi_height = GRID_RECT[3]
    result = empty_measurement()
    step = max(1, int(step))
    # D5: MIN_TRACE_POINTS 判定改为等效满采样点数（step 越大采样越稀，
    # 等效点数 = 实际点数 x (step/3)^2），避免 counts 模式误判 NO_TRACE。
    if points is None or len(points) * (
            float(step) / TRACE_SAMPLE_STEP) ** 2 < MIN_TRACE_POINTS:
        return result
    result["trace_pixels"] = len(points) * step * step
    bounds = robust_trace_bounds(points, step)
    if bounds is None:
        result["invalid_reason"] = "NO_TRACE"
        return result
    low_x, high_x, low_y, high_y = bounds
    result["pixel_bounds"] = bounds
    result["contacts_local"] = []
    px_per_div_x = float(roi_width) / GRID_DIVISIONS_X
    px_per_div_y = float(roi_height) / GRID_DIVISIONS_Y
    span_x_div = (high_x - low_x) / px_per_div_x
    span_y_div = (high_y - low_y) / px_per_div_y
    center_x_div = (
        0.5 * (low_x + high_x) - 0.5 * (roi_width - 1)
    ) / px_per_div_x
    center_y_div = (
        0.5 * (roi_height - 1) - 0.5 * (low_y + high_y)
    ) / px_per_div_y
    bounds_div = (
        (low_x - 0.5 * (roi_width - 1)) / px_per_div_x,
        (high_x - 0.5 * (roi_width - 1)) / px_per_div_x,
        (0.5 * (roi_height - 1) - high_y) / px_per_div_y,
        (0.5 * (roi_height - 1) - low_y) / px_per_div_y,
    )
    result.update({
        "span_x_div": span_x_div,
        "span_y_div": span_y_div,
        "center_x_div": center_x_div,
        "center_y_div": center_y_div,
        "bounds_div": bounds_div,
        # 与 fast_trace_stats() 相同的填充率语义：细静态轨迹得分低，
        # 旋转包络得分高。每个阶段都提供该值，叠加层可随时显示，
        # SEARCH 门限在快速与完整测量间共用同一口径。
        "fill_ratio": result["trace_pixels"] / float(
            max(1, (high_x - low_x) * (high_y - low_y))
        ),
    })
    # v2 ROTATION_LOCK 主轴角：基于 div 空间 PCA 主轴（theta），
    # 不依赖四边计数有效性，旋转包络/静态图形都可输出；
    # div 空间避免了 X/Y 非均匀像素缩放的夹角失真。
    div_points = points_to_divisions(points, roi_width, roi_height)
    moments = _point_moments(div_points)
    if moments is not None:
        result["principal_axis_angle_deg"] = math.degrees(moments[7])
    else:
        result["principal_axis_angle_deg"] = None

    if mode == "span":
        result["invalid_reason"] = "SPAN_ONLY"
        return result

    # 水平 X 单独预检刻意只有很小的 Y 跨度。它仍返回稳健的 X span，
    # 但不是可用于搜索的轨迹。
    if mode == 'shape':
        if span_x_div < MIN_TRACE_SPAN_DIV or span_y_div < MIN_TRACE_SPAN_DIV:
            result['invalid_reason'] = 'TRACE_SPAN_TOO_SMALL'
            return result
        edge_cut = trace_edge_cut_div(points, roi_width, roi_height)
        result['edge_cut_div'] = edge_cut
        # v3.1（根因4修复）：满幅免疫。8div 满屏（要求4的 8div 档、
        # 4Vpp@0.5V/div 输入）是合法目标态，轨迹贴边属正常；仅当图形
        # 未达满幅却贴边（真过幅，顶部/底部被削平）才置 edge_clipped。
        # 旧实现无条件否决：4Vpp 输入的形状永远不 pass -> LOCKED 永不
        # 达成（666 数据 1k/2k 组档位正确仍失败的主因，
        # reason=EDGE_CLIPPED）。
        _full_span = (span_x_div >= GRID_DIVISIONS_X - 0.5 or
                      span_y_div >= GRID_DIVISIONS_Y - 0.5)
        result['edge_clipped'] = bool(
            edge_cut > EDGE_CUT_SPREAD_DIV and not _full_span)
        result['valid'] = True
        result['invalid_reason'] = None
    elif span_x_div < MIN_TRACE_SPAN_DIV or span_y_div < MIN_TRACE_SPAN_DIV:
        result["invalid_reason"] = "TRACE_SPAN_TOO_SMALL"
    else:
        sides = {}
        for side in ("left", "right", "top", "bottom"):
            sides[side] = multiscale_side_count(points, bounds, side)
            result[side + "_scale_counts"] = sides[side]["scale_counts"]
            result[side] = sides[side]["count"]
            item = sides[side]
            if item["valid"]:
                result["contacts_local"].extend(item["contact_positions"])
        if not all(sides[side]["valid"] for side in sides):
            result["invalid_reason"] = "PEAK_MERGED" if any(
                item["reason"] == "PEAK_MERGED" for item in sides.values()
            ) else "CONTACT_POSITION_UNSTABLE"
        else:
            left = sides["left"]["count"]
            right = sides["right"]["count"]
            top = sides["top"]["count"]
            bottom = sides["bottom"]["count"]
            valid, reason, ratio_y, ratio_x = strict_ratio_from_counts(
                left, right, top, bottom
            )
            result.update({
                "valid": valid,
                "strict_pair_valid": valid,
                "invalid_reason": reason,
                "left": left,
                "right": right,
                "top": top,
                "bottom": bottom,
                "ratio_x": ratio_x,
                "ratio_y": ratio_y,
            })

    if mode == "counts" or not result["valid"]:
        return result

    unavailable = {"error": 999.0, "score": 0.0, "pass": False}
    diagonal = diagonal_shape_metrics(div_points) if target_shape in (None, 1) else unavailable
    circle = circle_shape_metrics(div_points) if target_shape in (None, 2) else unavailable
    infinity = infinity_shape_metrics(div_points) if target_shape in (None, 3) else unavailable
    result.update({
        "score_diagonal": diagonal["score"],
        "score_circle": circle["score"],
        "score_infinity": infinity["score"],
        "error_diagonal": diagonal["error"],
        "error_circle": circle["error"],
        "error_infinity": infinity["error"],
        "diagonal_pass": diagonal["pass"],
        "circle_pass": circle["pass"],
        "infinity_pass": infinity["pass"],
        "radial_cv": circle.get("radial_cv", 999.0),
        "correlation": circle.get("correlation", 0.0),
        "mirror_iou_lr": infinity.get("mirror_iou_lr", 0.0),
        "mirror_iou_tb": infinity.get("mirror_iou_tb", 0.0),
        "center_cross_distance_div": infinity.get(
            "center_cross_distance_div", 999.0
        ),
        "lobe_area_imbalance": infinity.get(
            "lobe_area_imbalance", 1.0
        ),
    })
    # 过幅（轨迹穿出屏幕）时，可见部分仍是"缩小版"的圆/∞ 形，
    # 各项形状指标都会通过——必须用边缘截断证据否决。
    if result.get("edge_clipped"):
        result["diagonal_pass"] = False
        result["circle_pass"] = False
        result["infinity_pass"] = False
        result["invalid_reason"] = "EDGE_CLIPPED"
        result["shape"] = "clipped"
        result["shape_confidence"] = 0.0
        return result
    candidates = (
        (diagonal["score"], "diagonal", diagonal["pass"]),
        (circle["score"], "ellipse", circle["pass"]),
        (infinity["score"], "infinity", infinity["pass"]),
    )
    best = max(candidates)
    result["shape"] = best[1]
    result["shape_confidence"] = best[0]
    return result

def measure_image(img):
    points = collect_yellow_points(img, GRID_RECT)
    measurement = analyze_trace_points(points, GRID_RECT[2], GRID_RECT[3])
    points = None
    return measurement

def shape_error_for_target(measurement, target_shape):
    if not measurement:
        return 999.0
    key = {
        1: "error_diagonal",
        2: "error_circle",
        3: "error_infinity",
    }[int(target_shape)]
    if key in measurement:
        return float(measurement.get(key, 999.0))
    score_key = {
        1: "score_diagonal",
        2: "score_circle",
        3: "score_infinity",
    }[int(target_shape)]
    return max(0.0, 1.0 - float(measurement.get(score_key, 0.0)))

def target_shape_ok(measurement, target_shape):
    if not measurement:
        return False
    explicit_key = {
        1: "diagonal_pass",
        2: "circle_pass",
        3: "infinity_pass",
    }[int(target_shape)]
    if explicit_key in measurement:
        return bool(measurement.get(explicit_key))
    expected_name = {1: "diagonal", 2: "ellipse", 3: "infinity"}[
        int(target_shape)
    ]
    return bool(
        measurement.get("shape") == expected_name
        and float(measurement.get("shape_confidence", 0.0)) >= 0.75
    )


def target_ratio_matches(measurement, target_shape):
    if not measurement.get('valid'):
        return False
    expected = (1, 2) if int(target_shape) == 3 else (1, 1)
    return (
        int(measurement.get('ratio_x', 0)),
        int(measurement.get('ratio_y', 0)),
    ) == expected


class DoneEvent:
    def __init__(self, sequence, input_frequency_millihz):
        self.sequence = int(sequence)
        self.input_frequency_millihz = int(input_frequency_millihz)


class LockedEvent:
    def __init__(self, sequence, input_frequency_millihz):
        self.sequence = int(sequence)
        self.input_frequency_millihz = int(input_frequency_millihz)

class FailedEvent:
    def __init__(self, sequence, error_code, reason):
        self.sequence = int(sequence)
        self.error_code = int(error_code)
        self.reason = str(reason)

class Requirement5Controller:
    WAIT_START = "WAIT_START"
    MEASURE_X_AMPLITUDE = "MEASURE_X_AMPLITUDE"
    RAMP_ESTIMATE = "RAMP_ESTIMATE"
    ROTATION_LOCK = "ROTATION_LOCK"
    LOCAL_SWEEP = "LOCAL_SWEEP"          # v3.1：错档纠错局部网格扫描
    PHASE_FINE = "PHASE_FINE"
    AMPLITUDE_FINE = "AMPLITUDE_FINE"
    HOLD = "HOLD"
    DONE = "DONE"
    FAILED = "FAILED"

    ERROR_UART_TIMEOUT = 1
    ERROR_CONTROL_TIMEOUT = 2
    ERROR_BAD_SHAPE = 3
    ERROR_FREQUENCY_NOT_FOUND = 4
    ERROR_AMPLITUDE = 5
    ERROR_SHAPE_NOT_FOUND = 6
    ERROR_F407 = 10

    def __init__(self, hold_ms=HOLD_REQUIRED_MS,
                 control_timeout_ms=CONTROL_TIMEOUT_MS,
                 ack_timeout_ms=UART_ACK_TIMEOUT_MS,
                 max_ack_retries=UART_MAX_ACK_RETRIES):
        self.hold_ms = int(hold_ms)
        self.control_timeout_ms = int(control_timeout_ms)
        self.ack_timeout_ms = int(ack_timeout_ms)
        self.max_ack_retries = int(max_ack_retries)
        self.sequence = 0
        self._reset_cycle()

    def _reset_cycle(self):
        self.state = self.WAIT_START
        self.failure_code = 0
        self.failure_reason = ""
        self.target_shape = 0
        self.started_ms = None
        self.last_command = None
        self.waiting_for_applied = False
        self.ack_started_ms = None
        self.ack_retries = 0
        self.applied_ms = None
        self.control_probe_id = 0x7FFF
        self.target_span_div = 0.0
        self.x_span_samples = []
        self.x_measure_frames = 0
        self.settle_discarded = 0
        self.locked_frequency_millihz = 0
        self.phase_deg = 0.0
        self.amplitude_mv = amplitude_feedforward(100, 8.0)
        self.amplitude_samples = []
        self.amplitude_corrections = 0
        self.amplitude_invalid_frames = 0
        self.hold_since_ms = None
        self.hard_fail_frames = 0
        # 装置自身当前输出频率（DDS 内部状态，非信号源频率），仅作上下文记录
        self.output_freq_millihz = 0
        self.last_output_millihz = 0
        self.rotation_samples = []
        self.rotation_rounds = 0
        self.phase_good_frames = 0
        self.phase_amplitude_checked = False
        self.phase_scan_candidates = []
        self.phase_scan_cursor = 0
        self.phase_scan_active = False
        self.phase_scan_refining = False
        self.phase_scan_best_error = 999.0
        self.phase_scan_best_deg = 0.0
        self.phase_scan_best_pass = False
        self.hold_fill_reference = None
        self.hold_geometry_reference = None
        self.last_hold_sample_ms = None
        # v3：固定周期短斜坡粗测频（RAMP_ESTIMATE）
        self.ramp_active = False
        self.ramp_shot_frames = 0
        self.ramp_iterations = 0
        self.ramp_saw_hz = 0
        self.ramp_cycle_estimates = []
        self.ramp_rejected_estimates = []
        # v3.1：局部网格纠错扫描（LOCAL_SWEEP，根因2修复）
        self.local_sweep_count = 0
        self.sweep_center_grid = 0
        self.sweep_center_fill = 9.9
        self.sweep_measured = {}
        self.sweep_best_grid = 0
        self.sweep_best_fill = 9.9
        self.sweep_stage = 0        # 0=V型快查(±2) 1=粗扫(±4..±12 步2) 2=细化(±1)
        self.sweep_candidate_list = []
        self.sweep_cursor = 0
        self.sweep_fill_samples = []

    def _next_sequence(self):
        self.sequence = (self.sequence + 1) & 0xFFFF
        if self.sequence == 0:
            self.sequence = 1
        return self.sequence

    def _next_control_probe_id(self):
        self.control_probe_id += 1
        if self.control_probe_id > 0xFFFF:
            self.control_probe_id = 0x8000
        return self.control_probe_id

    def _output_grid_max(self):
        """输出频率上限（2:1 目标允许到 200kHz）。"""
        return OUTPUT_GRID_MAX_2TO1 if self.target_shape == 3 else SWEEP_GRID_MAX

    def _issue_millihz(self, frequency_millihz, phase_deg, amplitude_mv,
                       probe_id, now_ms):
        frequency_millihz = int(clamp(
            round(frequency_millihz), 1000,
            self._output_grid_max() * 100000
        ))
        self.phase_deg = wrap_phase_deg(phase_deg)
        self.amplitude_mv = int(clamp(
            round(amplitude_mv), 0, DDS_AMPLITUDE_MAX_MV
        ))
        self.last_command = {
            "kind": "SET",
            "sequence": self._next_sequence(),
            "probe_id": int(probe_id),
            "frequency_grid": int(round(frequency_millihz / 100000.0)),
            "frequency_millihz": frequency_millihz,
            "phase_millideg": wrap_phase_millideg(
                round(self.phase_deg * 1000.0)
            ),
            "amplitude_mv": self.amplitude_mv,
            "shape": self.target_shape,
        }
        self.waiting_for_applied = True
        self.ack_started_ms = int(now_ms)
        self.ack_retries = 0
        self.applied_ms = None
        self.last_output_millihz = int(frequency_millihz)
        return self.last_command

    def _issue_grid(self, frequency_grid, phase_deg, amplitude_mv,
                    probe_id, now_ms):
        return self._issue_millihz(
            int(frequency_grid) * 100000, phase_deg, amplitude_mv,
            probe_id, now_ms
        )

    def start(self, target_shape, freq_millihz, now_ms):
        """一键启动：shape=1/2/3，freq_millihz 为装置自身当前输出频率（mHz）。

        题目要求5（合规约束）：断信号源与装置的连接、无电气耦合、纯摄像头，
        频率点不得在装置中设置/传入。freq_millihz 只是装置 DDS 自己正在输出
        的频率（内部状态，非信号源频率），仅作斜坡测频前的上下文记录；
        MEASURE_X 收敛后走 RAMP 斜坡粗测频（估频不依赖 freq_millihz）。
        """
        if int(target_shape) not in (1, 2, 3):
            return self._fail(
                self.ERROR_BAD_SHAPE, "target shape must be 1, 2, or 3"
            )
        saved_sequence = self.sequence
        self._reset_cycle()
        self.sequence = saved_sequence
        self.target_shape = int(target_shape)
        self.output_freq_millihz = int(freq_millihz)
        self.started_ms = int(now_ms)
        self.state = self.MEASURE_X_AMPLITUDE
        print("[启动] 模式=%d（%s） 当前输出=%.3fkHz 目标幅度待测" % (
            self.target_shape,
            {1: "对角线", 2: "圆", 3: "无穷形"}[self.target_shape],
            self.output_freq_millihz / 1.0e6))
        # START is preceded by F407 switching PA4 to the sawtooth output.
        # Do not send a placeholder SET here: MEASURE_X 收敛后必发 $RAMP。
        self.last_command = None
        self.waiting_for_applied = False
        self.ack_started_ms = None
        self.ack_retries = 0
        self.applied_ms = int(now_ms)
        return None

    def _issue_ramp(self, saw_hz, probe_id, now_ms):
        """发 $RAMP,seq,probe,saw_hz,amp_mv：F407 切 PA4 DAC 输出锯齿斜坡。"""
        self.last_command = {
            "kind": "RAMP",
            "sequence": self._next_sequence(),
            "probe_id": int(probe_id),
            "saw_hz": int(saw_hz),
            "amplitude_mv": int(RAMP_AMP_MV),
        }
        self.waiting_for_applied = True
        self.ack_started_ms = int(now_ms)
        self.ack_retries = 0
        self.applied_ms = None
        self.ramp_saw_hz = int(saw_hz)
        return self.last_command

    def _begin_ramp_estimate(self, now_ms):
        """MEASURE_X 收敛后：发固定周期短斜坡探测，估频后直接精确 SET。

        只在 RAMP_ENABLED 时使用；任何斜坡失败（无轨迹/周期越界/F407
        拒绝/ACK 超时）都进入明确 FAILED 状态，不偷偷回退 v2 扫频。
        """
        self.ramp_active = True
        self.ramp_shot_frames = 0
        self.ramp_iterations = 0
        self.ramp_saw_hz = 0
        self.ramp_cycle_estimates = []
        self.ramp_rejected_estimates = []
        self.state = self.RAMP_ESTIMATE
        print("[斜坡] RAMP 粗测频启动：初始斜坡 %d Hz（T=%.3f ms）" % (
            RAMP_SAW_HZ, 1000.0 / RAMP_SAW_HZ))
        return self._issue_ramp(
            RAMP_SAW_HZ, self._next_control_probe_id(), now_ms
        )

    def _ramp_estimate_step(self, measurement, now_ms):
        """斜坡探测帧分析：累加周期数估计，满 3 次取中值 -> 收敛。

        无轨迹/低质量帧计失败帧，超 RAMP_SHOT_MAX_FRAMES 换斜坡档重试。
        D12：每帧打印示波器观测到的输入波形频率（fx=N*斜坡频率），
        便于现场用串口日志核对斜坡扫描过程。
        """
        n_cycles = measurement.get("ramp_cycles")
        quality = float(measurement.get("ramp_quality", 0.0))
        if n_cycles is None:
            print("[斜坡] 帧%d 斜坡=%dHz 无轨迹（示波器未检出正弦/锯齿）" % (
                self.ramp_shot_frames + 1, self.ramp_saw_hz))
            # 完全无轨迹：快慢交替找可见性（低频需慢斜坡放大周期数）
            self.ramp_shot_frames += 1
            if self.ramp_shot_frames >= RAMP_SHOT_MAX_FRAMES:
                return self._ramp_shot_failed(now_ms)
            return None
        fx_khz = float(n_cycles) * float(self.ramp_saw_hz) / 1000.0
        print("[斜坡] 帧%d 斜坡=%dHz N=%.2f 质量=%.3f 示波器波形fx≈%.3fkHz" % (
            self.ramp_shot_frames + 1, self.ramp_saw_hz,
            n_cycles, quality, fx_khz))
        if quality < RAMP_QUALITY_GATE:
            # 有估计但拟合质量差：真实 N 大概率超出 DFT 扫描范围（欠采样/
            # 泄漏），换快斜坡把 N 压回可测区；不浪费慢档（慢档只会更糟）。
            self.ramp_shot_frames += 1
            if self.ramp_shot_frames >= RAMP_SHOT_MAX_FRAMES:
                print("[斜坡] 质量差 %.3f（N=%.1f）连续 %d 帧 -> 换快斜坡复测" % (
                    quality, n_cycles, RAMP_SHOT_MAX_FRAMES))
                return self._ramp_retune(n_cycles, now_ms)
            return None
        self.ramp_cycle_estimates.append((float(n_cycles), quality))
        self.ramp_shot_frames += 1
        if len(self.ramp_cycle_estimates) < RAMP_MEDIAN_FRAMES:
            return None
        ests = sorted(item[0] for item in self.ramp_cycle_estimates)
        n_median = ests[len(ests) // 2]
        spread = ests[-1] - ests[0]
        mad_values = sorted(abs(value - n_median) for value in ests)
        mad = mad_values[len(mad_values) // 2]
        spread_limit = max(0.6, n_median * RAMP_MAX_RELATIVE_SPREAD)
        mad_limit = max(0.25, n_median * RAMP_MAX_RELATIVE_MAD)
        if spread > spread_limit or mad > mad_limit:
            print("[斜坡] 估计不一致 N中值=%.2f spread=%.2f MAD=%.2f，拒绝本档" % (
                n_median, spread, mad))
            self.ramp_cycle_estimates = []
            self.ramp_shot_frames = 0
            if self.ramp_saw_hz >= RAMP_UNSTABLE_SAW_HZ:
                return self._ramp_retune(n_median, now_ms)
            # 1 kHz 的低频档可能只是暂态/遮挡，先再采一组；连续两组
            # 不一致再切到更快的档，避免一次坏帧立即改变输出频率。
            self.ramp_rejected_estimates.append((n_median, spread, mad))
            if len(self.ramp_rejected_estimates) >= 2:
                return self._ramp_retune(n_median, now_ms)
            return None
        self.ramp_rejected_estimates = []
        return self._ramp_accept_cycles(n_median, now_ms)

    def _ramp_accept_cycles(self, n_cycles, now_ms):
        """周期数 -> fx=N/T_ramp -> 100Hz 网格 -> 直接发精确 SET 锁频。

        D1 修复：DFT 估计顶到扫描上限附近（>=RAMP_SUSPECT_CYCLES）时，
        该值可能来自欠采样混叠（实测 N=100 -> 60.585 顶在 62 附近），
        不可直接 accept，强制换快斜坡档复测；正常区间才锁频。
        D12 修复：N>=RAMP_CONFIRM_CYCLES 时（每周期 <~12px，欠采样边缘）
        即使拟合质量好也换快斜坡复核，把 N 压到 <25 再 accept，消除
        50-70kHz 单次估计分辨率不足导致的锁频漂移。
        """
        # N>25 时每周期 <12.8px 欠采样；N=25（12.8px/周期）本身已达标
        if n_cycles > RAMP_CONFIRM_CYCLES:
            if n_cycles >= RAMP_SUSPECT_CYCLES:
                print("[斜坡] N=%.2f 顶到 DFT 上限附近（泄漏嫌疑），换快斜坡复测" % n_cycles)
            else:
                print("[斜坡] N=%.2f 偏密（每周期<12px 欠采样），换快斜坡复核" % n_cycles)
            return self._ramp_retune(n_cycles, now_ms)
        if n_cycles < RAMP_MIN_CYCLES or n_cycles > RAMP_MAX_CYCLES:
            return self._ramp_retune(n_cycles, now_ms)
        fx_hz = n_cycles * float(self.ramp_saw_hz)
        # D13 修复：输入识别严格限定 1kHz~100kHz、100Hz 网格（991 个点）。
        # 估频连续值归整到 100Hz；超范围（如 100.5kHz）clamp 到边界，
        # 不产生第 992 个值。
        input_grid = int(clamp(
            round(fx_hz / float(RAMP_GRID_ROUND)),
            SWEEP_GRID_MIN, SWEEP_GRID_MAX))
        ratio = 2 if self.target_shape == 3 else 1
        output_grid = int(clamp(
            input_grid * ratio, SWEEP_GRID_MIN, self._output_grid_max(),
        ))
        self.ramp_active = False
        self.locked_frequency_millihz = output_grid * 100000
        self.amplitude_mv = amplitude_feedforward(output_grid, self.target_span_div)
        self.rotation_samples = []
        self.rotation_rounds = 0
        self.state = self.ROTATION_LOCK
        # 网格边界距离（占 100Hz 步进的百分比）：<25% 表示归整可靠
        _nearest = round(fx_hz / float(RAMP_GRID_ROUND)) * float(RAMP_GRID_ROUND)
        _edge_pct = abs(fx_hz - _nearest) / float(RAMP_GRID_ROUND) * 100.0
        print("[斜坡] 复核通过 N=%.2f 斜坡=%dHz -> fx=%.3f kHz（距网格边界%.0f%%）-> 网格 %d 档，直接精确 SET 输出 %d 档" % (
            n_cycles, self.ramp_saw_hz, fx_hz / 1000.0, _edge_pct,
            input_grid, output_grid))
        return self._issue_grid(
            output_grid, 0.0, self.amplitude_mv,
            self._next_control_probe_id(), now_ms
        )

    def _ramp_retune(self, n_cycles, now_ms):
        """周期数越界/泄漏嫌疑：N 太小 -> 斜坡变慢（窗口内周期数变多）；
        N 太大或顶到 DFT 上限 -> 斜坡变快（窗口内周期数变少）。
        迭代上限后明确失败。"""
        if self.ramp_iterations >= RAMP_MAX_ITERATIONS:
            return self._ramp_fallback(now_ms)
        self.ramp_iterations += 1
        if n_cycles < RAMP_MIN_CYCLES:
            new_saw = max(100, self.ramp_saw_hz // 2)
        else:
            # 覆盖 N>RAMP_MAX_CYCLES 与 N>=RAMP_SUSPECT_CYCLES（泄漏嫌疑）两分支
            new_saw = min(10000, self.ramp_saw_hz * 2)
        if new_saw == self.ramp_saw_hz:
            return self._ramp_fallback(now_ms)
        self.ramp_shot_frames = 0
        self.ramp_cycle_estimates = []
        self.ramp_rejected_estimates = []
        print("[斜坡] N=%.2f 越界 -> 换斜坡 %d Hz 重测" % (n_cycles, new_saw))
        return self._issue_ramp(
            new_saw, self._next_control_probe_id(), now_ms
        )

    def _ramp_shot_failed(self, now_ms):
        """斜坡档连续无轨迹：快/慢交替换档重试，迭代上限后明确失败。

        D1 修复(2026-10-22)：原实现 iter 为偶数时 new_saw=base 换回
        同一斜坡，白耗 5 帧；改为严格快慢交替（快档测高频、慢档测低频）。
        """
        if self.ramp_iterations >= RAMP_MAX_ITERATIONS:
            return self._ramp_fallback(now_ms)
        self.ramp_iterations += 1
        base = RAMP_SAW_HZ if self.ramp_saw_hz <= 0 else self.ramp_saw_hz
        if self.ramp_iterations % 2 == 1:
            new_saw = min(10000, base * 2) if base < 5000 else max(100, base // 2)
        else:
            new_saw = max(100, base // 2) if base > 250 else min(10000, base * 2)
        if new_saw == self.ramp_saw_hz:
            return self._ramp_fallback(now_ms)
        self.ramp_shot_frames = 0
        self.ramp_cycle_estimates = []
        self.ramp_rejected_estimates = []
        print("[斜坡] 无轨迹 %d 帧 -> 换斜坡 %d Hz 重试" % (
            RAMP_SHOT_MAX_FRAMES, new_saw))
        return self._issue_ramp(
            new_saw, self._next_control_probe_id(), now_ms
        )

    def _ramp_fallback(self, now_ms):
        """斜坡测频失败 -> 进入明确 FAILED 状态（上报 $FAILED，不回退扫频）。"""
        self.ramp_active = False
        print("[斜坡] 测频失败 -> 进入明确失败状态（FAILED）")
        return self._fail(
            self.ERROR_FREQUENCY_NOT_FOUND,
            "sawtooth frequency estimate failed; sweep disabled",
        )

    # ================================================================
    # v3.1：LOCAL_SWEEP 局部网格纠错扫描（根因2修复）
    # ================================================================

    def _begin_local_sweep(self, now_ms, trigger_fill):
        """fill 门控触发的局部网格扫描（RAMP_LOCAL_SWEEP 落地）。

        旋转锁频的 rotation_delta_hz 只在小 Δf 下有效；错档 ≥1 档
        （≥100Hz）时图形转速超出角速度测量的可测范围，接触点/形状确认
        永不通过 -> 10s 超时（666 数据 10kHz 组：锁定 10.300kHz 后角速度
        -81.3deg/s 被判 Δf=-0.45Hz，实际差 300Hz）。改用 fill_ratio 判
        "静态细轨迹(~0.05) vs 旋转实心包络(~0.5)"（分界 0.35，两侧裕度
        数倍），对当前输出档做 V 型快查 + 粗扫 + ±1 细化，把网格拉回
        正确档后回正常收敛链路。判据与幅度/相位无关，全频段同口径。
        """
        center = int(round(self.locked_frequency_millihz / 100000.0))
        center = int(clamp(center, SWEEP_GRID_MIN, self._output_grid_max()))
        self.local_sweep_count += 1
        self.sweep_center_grid = center
        self.sweep_center_fill = float(trigger_fill)
        self.sweep_measured = {center: float(trigger_fill)}
        self.sweep_best_grid = center
        self.sweep_best_fill = float(trigger_fill)
        self.sweep_stage = 0
        self.sweep_candidate_list = [center - 2, center + 2]
        self.sweep_cursor = 0
        self.sweep_fill_samples = []
        self.state = self.LOCAL_SWEEP
        print("[网格扫] fill=%.2f 超门限 -> 图形旋转=错档嫌疑，启动局部扫描 中心=%d档（±%d档）" % (
            trigger_fill, center, RAMP_LOCAL_SWEEP_RANGE_GRID))
        return self._issue_grid(
            self.sweep_candidate_list[0], 0.0, self.amplitude_mv,
            self._next_control_probe_id(), now_ms
        )

    def _local_sweep_step(self, measurement, now_ms):
        """LOCAL_SWEEP 帧处理：每档收 RAMP_SWEEP_FRAMES 帧 fill 取中值。

        无 fill 的帧（NO_TRACE 等空测量）计 9.9 视为最差点，不影响选优。
        """
        grid = self.sweep_candidate_list[self.sweep_cursor]
        fill = float(measurement.get("fill_ratio", 9.9))
        self.sweep_fill_samples.append(fill)
        if len(self.sweep_fill_samples) < RAMP_SWEEP_FRAMES:
            return None
        fill_med = median(self.sweep_fill_samples)
        self.sweep_fill_samples = []
        self.sweep_measured[grid] = fill_med
        if fill_med < self.sweep_best_fill:
            self.sweep_best_fill = fill_med
            self.sweep_best_grid = grid
        self.sweep_cursor += 1
        if self.sweep_cursor < len(self.sweep_candidate_list):
            return self._issue_grid(
                self.sweep_candidate_list[self.sweep_cursor], 0.0,
                self.amplitude_mv, self._next_control_probe_id(), now_ms
            )
        return self._local_sweep_stage_done(now_ms)

    def _local_sweep_stage_done(self, now_ms):
        """单轮候选测完：V 型判据 / 粗扫 / 细化 的推进与收口。"""
        center = self.sweep_center_grid
        if self.sweep_stage == 0:
            # V 型快查：中心档已是静态细轨迹且比两侧低 -> 网格正确，
            # 问题在相位/幅度 -> 回正常收敛链路（不换档）。
            left = self.sweep_measured.get(center - 2, 9.9)
            right = self.sweep_measured.get(center + 2, 9.9)
            if (
                self.sweep_center_fill < RAMP_SWEEP_STATIC_FILL
                and self.sweep_center_fill < min(left, right) - RAMP_SWEEP_FILL_MARGIN
            ):
                return self._finish_local_sweep(
                    now_ms, center, "V型快查: 中心档fill最低且静态")
            # 粗扫 ±(4..12) 步 2（±2 已在 V 型快查中测过）
            cand = []
            for off in range(4, RAMP_LOCAL_SWEEP_RANGE_GRID + 1, 2):
                for g in (center + off, center - off):
                    g = int(clamp(g, SWEEP_GRID_MIN, self._output_grid_max()))
                    if g not in self.sweep_measured and g not in cand:
                        cand.append(g)
            if not cand:
                return self._finish_local_sweep(
                    now_ms, self.sweep_best_grid, "粗扫无新点")
            self.sweep_stage = 1
            self.sweep_candidate_list = cand
            self.sweep_cursor = 0
            return self._issue_grid(
                cand[0], 0.0, self.amplitude_mv,
                self._next_control_probe_id(), now_ms
            )
        if self.sweep_stage == 1:
            # 细化：最优档 ±1（已测过的跳过）
            cand = []
            for g in (self.sweep_best_grid - 1, self.sweep_best_grid + 1):
                g = int(clamp(g, SWEEP_GRID_MIN, self._output_grid_max()))
                if g not in self.sweep_measured and g not in cand:
                    cand.append(g)
            if not cand:
                return self._finish_local_sweep(
                    now_ms, self.sweep_best_grid, "最优档邻点已测")
            self.sweep_stage = 2
            self.sweep_candidate_list = cand
            self.sweep_cursor = 0
            return self._issue_grid(
                cand[0], 0.0, self.amplitude_mv,
                self._next_control_probe_id(), now_ms
            )
        return self._finish_local_sweep(
            now_ms, self.sweep_best_grid, "细化完成")

    def _finish_local_sweep(self, now_ms, grid, note):
        """扫描收口：回最优档，重置收敛链路（ROTATION_LOCK 起步）。"""
        grid = int(clamp(grid, SWEEP_GRID_MIN, self._output_grid_max()))
        self.locked_frequency_millihz = grid * 100000
        self.rotation_samples = []
        self.rotation_rounds = 0
        self.phase_good_frames = 0
        self.phase_amplitude_checked = False
        self.phase_scan_active = False
        self.state = self.ROTATION_LOCK
        print("[网格扫] %s -> 选定 %d 档（fill=%.2f），回旋转锁频链路" % (
            note, grid, self.sweep_best_fill))
        return self._issue_grid(
            grid, 0.0, self.amplitude_mv,
            self._next_control_probe_id(), now_ms
        )


    def _rotation_slope_deg_per_sec(self):
        """ROTATION_LOCK 主轴角序列 -> 角速度(°/s)。

        角度先 unwrap 再取首末差，除以时间跨度；样本不足返回 0。
        """
        if len(self.rotation_samples) < 2:
            return 0.0
        first_stamp = self.rotation_samples[0][1]
        last_stamp = self.rotation_samples[-1][1]
        dt_ms = ticks_diff(last_stamp, first_stamp)
        if dt_ms <= 0:
            return 0.0
        angles = [item[0] for item in self.rotation_samples]
        unwrapped = unwrap_axial_degrees(angles)
        return float(unwrapped[-1] - unwrapped[0]) * 1000.0 / float(dt_ms)

    def _reported_input_frequency_millihz(self):
        ratio = 2 if self.target_shape == 3 else 1
        grid = int(round(
            self.locked_frequency_millihz / (100000.0 * ratio)
        ))
        return int(clamp(grid * 100000, 1000000, 100000000))

    def _start_phase_scan(self, now_ms):
        # 2026-10-22 撤销 D8 跳过：shape=1/2 对相位并非"不敏感"。
        # 1:1 双正弦的相关性 corr=cos(φ)：φ=0 -> 对角线，φ=90 -> 圆。
        # 输入源与 AD9959 异步（赛题要求 5 禁止耦合），锁定瞬间的相对相位
        # φ0 未知，直接发 phase=0 时圆/对角线各有约一半概率永远不 pass，
        # PHASE_FINE 会一直耗到 10s 控制超时。必须实扫寻找相对相位。
        self.phase_good_frames = 0
        self.phase_scan_candidates = list(range(0, 360, PHASE_SCAN_STEP_DEG))
        self.phase_scan_cursor = 0
        self.phase_scan_active = True
        self.phase_scan_refining = False
        self.phase_scan_best_error = 999.0
        self.phase_scan_best_deg = 0.0
        self.phase_scan_best_pass = False
        return self._issue_millihz(
            self.locked_frequency_millihz, 0.0, self.amplitude_mv,
            self._next_control_probe_id(), now_ms
        )

    def _adjust_amplitude_for_span(self, span_y):
        """按实测 span_y 修正 DDS 幅度指令；返回是否实际改变。"""
        scale = clamp(self.target_span_div / span_y, 0.90, 1.10)
        adjusted = int(clamp(
            round(self.amplitude_mv * scale),
            DDS_AMPLITUDE_MIN_MV, DDS_AMPLITUDE_MAX_MV
        ))
        if adjusted == self.amplitude_mv:
            adjusted += 1 if span_y < self.target_span_div else -1
            adjusted = int(clamp(
                adjusted, DDS_AMPLITUDE_MIN_MV, DDS_AMPLITUDE_MAX_MV
            ))
        changed = adjusted != self.amplitude_mv
        self.amplitude_mv = adjusted
        return changed

    def _phase_amplitude_precheck(self, measurement, now_ms):
        """相位扫描前的幅度预收敛（只依赖 span_y，与相位/形状无关）。

        1:1 与 2:1 的 Lissajous 图 Y 向极值在一个完整周期内都会达到，
        所以 bbox span_y 与相对相位无关、始终等于器件输出幅度。形状
        pass 门限却要求 span_x≈span_y；幅度偏差 >0.12 div 时 PHASE_FINE
        永远不 pass，也就永远到不了 AMPLITUDE_FINE。这里先单独把
        span_y 收敛到目标，再开始相位扫描。
        """
        if not measurement.get("valid"):
            return None
        # 过幅时 span_y 被屏幕截断、恒等于目标，幅度闭环会误以为已收敛；
        # 边缘截断证据优先，直接按 10% 步长收小，最多 4 次。
        if measurement.get("edge_clipped") and self.amplitude_corrections < 4 and \
                self.target_span_div < GRID_DIVISIONS_Y - 0.5:
            # 满幅免疫(2026-10-22)：目标接近满幅（如 X=4V -> 8div）时，
            # ∞ 形左右 lobe / 噪声点会触发 edge_clipped 误判
            # （trace_edge_cut_div 实测 ∞ 形 8div=6.2div > 阈值 2.5），
            # 但"恰好满幅"与"过幅"在 span_y 上无法区分，无条件 ×0.9 连减
            # 会把输出从 4V 单调拉到 ~2.8V。目标不满幅时才用截断证据减幅。
            self.amplitude_mv = int(clamp(
                round(self.amplitude_mv * 0.90),
                DDS_AMPLITUDE_MIN_MV, DDS_AMPLITUDE_MAX_MV
            ))
            self.amplitude_corrections += 1
            print("[幅度] 预收敛 %d/4: 检测到出屏截断 %.1fdiv -> 指令 %d mV" % (
                self.amplitude_corrections,
                float(measurement.get("edge_cut_div", 0.0)),
                self.amplitude_mv))
            return self._issue_millihz(
                self.locked_frequency_millihz, self.phase_deg,
                self.amplitude_mv, self._next_control_probe_id(), now_ms
            )
        if measurement.get("edge_clipped"):
            # 目标满幅 + 截断证据：满幅贴边（正常目标态）而非过幅。
            # 不强制减幅，直接按 span_y 收敛（span≈目标 -> LOCKED）。
            print("[幅度] edge_clipped %.1fdiv 目标 %.1fdiv 满幅贴边，按 span 收敛" % (
                float(measurement.get("edge_cut_div", 0.0)),
                self.target_span_div))
        span_y = float(measurement.get("span_y_div", 0.0))
        if span_y <= 0.0:
            return None
        error = self.target_span_div - span_y
        if (
            abs(error) <= AMPLITUDE_CONVERGE_DIV
            or self.amplitude_corrections >= 4
        ):
            if abs(error) > AMPLITUDE_CONVERGE_DIV:
                print("[幅度] 预收敛 4 次后仍有 %.2f div 偏差，继续相位扫描" % (
                    error))
            else:
                print("[幅度] 预收敛 span_y=%.2f div (目标 %.2f)" % (
                    span_y, self.target_span_div))
            self.phase_amplitude_checked = True
            return self._start_phase_scan(now_ms)
        self._adjust_amplitude_for_span(span_y)
        self.amplitude_corrections += 1
        print("[幅度] 预收敛 %d/4: span_y=%.2f -> 指令 %d mV" % (
            self.amplitude_corrections, span_y, self.amplitude_mv))
        return self._issue_millihz(
            self.locked_frequency_millihz, self.phase_deg, self.amplitude_mv,
            self._next_control_probe_id(), now_ms
        )

    def _reject_current_candidate(self, now_ms):
        """旋转锁频未收敛 -> 明确 FAILED（RAMP 单链路，无 v2 换档）。"""
        self.phase_scan_active = False
        return self._fail(
            self.ERROR_SHAPE_NOT_FOUND,
            "rotation lock did not converge within rounds",
        )

    def _phase_scan_step(self, measurement, now_ms):
        passed = target_shape_ok(measurement, self.target_shape)
        error = shape_error_for_target(measurement, self.target_shape)
        if (
            error < self.phase_scan_best_error
            or (passed and not self.phase_scan_best_pass)
        ):
            self.phase_scan_best_error = error
            self.phase_scan_best_deg = self.phase_scan_candidates[
                self.phase_scan_cursor
            ]
        self.phase_scan_best_pass = self.phase_scan_best_pass or passed
        self.phase_scan_cursor += 1

        if self.phase_scan_cursor < len(self.phase_scan_candidates):
            phase = self.phase_scan_candidates[self.phase_scan_cursor]
            return self._issue_millihz(
                self.locked_frequency_millihz, phase, self.amplitude_mv,
                self._next_control_probe_id(), now_ms
            )

        if not self.phase_scan_refining:
            center = self.phase_scan_best_deg
            self.phase_scan_candidates = [
                wrap_phase_deg(center + offset)
                for offset in range(-15, 16, PHASE_REFINE_STEP_DEG)
            ]
            self.phase_scan_cursor = 0
            self.phase_scan_refining = True
            return self._issue_millihz(
                self.locked_frequency_millihz,
                self.phase_scan_candidates[0], self.amplitude_mv,
                self._next_control_probe_id(), now_ms
            )

        self.phase_scan_active = False
        if self.phase_scan_best_pass:
            self.phase_good_frames = 0
            return self._issue_millihz(
                self.locked_frequency_millihz, self.phase_scan_best_deg,
                self.amplitude_mv, self._next_control_probe_id(), now_ms
            )
        if self.rotation_rounds < ROTATION_MAX_ROUNDS:
            self.state = self.ROTATION_LOCK
            self.rotation_samples = []
            return None
        return self._reject_current_candidate(now_ms)

    def on_applied(self, sequence, probe_id, now_ms):
        if (
            not self.waiting_for_applied or self.last_command is None
            or int(sequence) != self.last_command["sequence"]
            or int(probe_id) != self.last_command["probe_id"]
        ):
            return False
        self.waiting_for_applied = False
        self.applied_ms = int(now_ms)
        self.settle_discarded = 0
        return True

    def measurement_ready(self, now_ms):
        """每帧预览调用一次；APPLIED 后丢弃稳定帧。"""
        if self.waiting_for_applied or self.applied_ms is None:
            return False
        settle_ms = POST_APPLIED_SETTLE_MS
        discard_frames = DISCARD_FRAMES_AFTER_APPLIED
        if self.state == self.RAMP_ESTIMATE:
            # RAMP 斜坡帧仍需丢掉切换后的首帧；否则继电器/DAC 残影会
            # 被行质心算法解释成一个合法的低频周期。
            settle_ms = min(POST_APPLIED_SETTLE_MS, SWEEP_POST_APPLIED_SETTLE_MS)
            discard_frames = RAMP_DISCARD_FRAMES_AFTER_APPLIED
        if self.state == self.LOCAL_SWEEP:
            # v3.1：网格扫描的 SET 只改 AD9959 频率（无继电器切换），
            # 100ms 稳定窗足够；每档仅 2 帧，settle 决定扫描总耗时。
            settle_ms = min(POST_APPLIED_SETTLE_MS, SWEEP_POST_APPLIED_SETTLE_MS)
        if ticks_diff(now_ms, self.applied_ms) < settle_ms:
            return False
        if self.settle_discarded < discard_frames:
            self.settle_discarded += 1
            return False
        return True

    def vision_lost(self):
        """未观测时间绝不能计入连续 HOLD；图形丢失回 ROTATION_LOCK 重锁。"""
        if self.state == self.HOLD:
            self.state = self.ROTATION_LOCK
            self.rotation_samples = []
            self.rotation_rounds = 0
            self.hold_since_ms = None
            self.hold_fill_reference = None
            self.hold_geometry_reference = None
            self.last_hold_sample_ms = None

    def on_error(self, sequence, probe_id, error_code):
        if (
            self.last_command is None
            or int(sequence) != self.last_command["sequence"]
            or int(probe_id) != self.last_command["probe_id"]
        ):
            return None
        if self.last_command.get("kind") == "RAMP":
            # F407 固件不认识 $RAMP：直接判 FAILED，不回退扫频。
            print("[斜坡] F407 拒绝 RAMP 命令 -> 进入明确失败状态（FAILED）")
            return self._fail(
                self.ERROR_F407,
                "F407 rejected RAMP; sweep is disabled",
            )
        code = self.ERROR_F407 + min(max(int(error_code), 1), 245)
        return self._fail(code, "F407 rejected SET")

    def _hold_step(self, measurement, now_ms):
        now_ms = int(now_ms)
        if (
            self.last_hold_sample_ms is not None
            and ticks_diff(now_ms, self.last_hold_sample_ms) > HOLD_MAX_SAMPLE_GAP_MS
        ):
            self.hold_since_ms = None
            self.hard_fail_frames = 0
        self.last_hold_sample_ms = now_ms

        geometry = (
            float(measurement.get('span_x_div', 0.0)),
            float(measurement.get('span_y_div', 0.0)),
            float(measurement.get('center_x_div', 999.0)),
            float(measurement.get('center_y_div', 999.0)),
        )
        good = bool(
            measurement.get('valid')
            and target_shape_ok(measurement, self.target_shape)
            and geometry[0] >= HOLD_SPAN_MIN_DIV
            and geometry[1] >= HOLD_SPAN_MIN_DIV
            and self.hold_geometry_reference is not None
        )
        if good:
            reference = self.hold_geometry_reference
            good = (
                abs(geometry[1] - self.target_span_div) <= HOLD_SPAN_DRIFT_DIV
                and abs(geometry[0] - reference[0]) <= HOLD_Y_REFERENCE_TOL_DIV
                and abs(geometry[2] - reference[2]) <= HOLD_CENTER_DRIFT_DIV
                and abs(geometry[3] - reference[3]) <= HOLD_CENTER_DRIFT_DIV
            )
        fill = float(measurement.get('fill_ratio', 1.0))
        if good and self.hold_fill_reference is not None:
            good = fill <= self.hold_fill_reference + HOLD_FILL_DRIFT_GATE

        if good:
            self.hard_fail_frames = 0
            if self.hold_since_ms is None:
                self.hold_since_ms = now_ms
            if ticks_diff(now_ms, self.hold_since_ms) >= self.hold_ms:
                self.state = self.DONE
                return DoneEvent(
                    self.sequence, self._reported_input_frequency_millihz()
                )
            return None

        self.hold_since_ms = None
        self.hard_fail_frames += 1
        if self.hard_fail_frames >= 3:
            self.state = self.ROTATION_LOCK
            self.rotation_samples = []
            self.rotation_rounds = 0
            self.hold_fill_reference = None
            self.hold_geometry_reference = None
            self.last_hold_sample_ms = None
        return None

    def on_measurement(self, measurement, now_ms):
        # MEASURE_X 阶段豁免 waiting_for_applied：start() 不发占位 SET，
        # X 幅度测量取自示波器输入轨迹（Y 已是 DAC 锯齿），与 ACK 无关；
        # 该阶段 waiting_for_applied 恒为 False，此豁免保留以防未来改动。
        if (
            self.waiting_for_applied
            and self.state != self.MEASURE_X_AMPLITUDE
        ) or self.state in (self.WAIT_START, self.FAILED):
            return None
        timeout = self._control_timeout(now_ms)
        if timeout is not None:
            return timeout

        if self.state == self.MEASURE_X_AMPLITUDE:
            self.x_measure_frames += 1
            span_x = float(measurement.get("span_x_div", 0.0))
            if span_x <= 0.0 or int(measurement.get("trace_pixels", 0)) <= 0:
                if self.x_measure_frames >= MEASURE_X_MAX_FRAMES:
                    return self._fail(self.ERROR_AMPLITUDE, "No usable X trace; check screen/background")
                return None
            self.x_span_samples.append(span_x)
            if len(self.x_span_samples) > 5:
                self.x_span_samples.pop(0)
            if len(self.x_span_samples) < 5:
                if self.x_measure_frames < MEASURE_X_MAX_FRAMES:
                    return None
                if len(self.x_span_samples) < 3:
                    return self._fail(self.ERROR_AMPLITUDE, "Insufficient usable X amplitude samples")
            if (max(self.x_span_samples) - min(self.x_span_samples) > 0.10
                    and self.x_measure_frames < MEASURE_X_MAX_FRAMES):
                return None
            self.target_span_div = float(median(self.x_span_samples))
            # D9 修复：Vpp 换算用 VOLTS_PER_DIV（默认 0.5）而非硬编码 0.5。
            # 现场必须把示波器 Y 通道锁定在该档位，否则 1~4Vpp 判定失真。
            x_vpp = self.target_span_div * VOLTS_PER_DIV
            if x_vpp < 0.9 or x_vpp > 4.1:
                return self._fail(
                    self.ERROR_AMPLITUDE,
                    "X amplitude %.3f Vpp is outside 1..4 Vpp" % x_vpp,
                )
            # 新主流程：MEASURE_X 收敛 -> 必发 $RAMP 斜坡粗测频；
            # 估频后直接发精确 $SET 锁频。RAMP 失败一律 FAILED。
            print("[幅度] 输入X幅度=%.2f div, %.2f Vpp, 输出幅度目标=%.2f div" % (
                self.target_span_div, x_vpp, self.target_span_div))
            if RAMP_ENABLED:
                return self._begin_ramp_estimate(now_ms)
            return self._fail(
                self.ERROR_FREQUENCY_NOT_FOUND,
                "RAMP_ENABLED is false; sweep is disabled",
            )

        if self.state == self.RAMP_ESTIMATE:
            return self._ramp_estimate_step(measurement, now_ms)

        if self.state == self.LOCAL_SWEEP:
            return self._local_sweep_step(measurement, now_ms)

        if self.state == self.ROTATION_LOCK:
            # v3.1（根因2修复）：fill 门控。错档（≥1 档=≥100Hz）时图形以
            # ≥50 转/s 旋转，30ms 曝光积分成实心包络 fill≥0.45；正确档为
            # 静态细轨迹 fill≤~0.15。fill 超门限即启动局部网格扫描拉回，
            # 不依赖 rotation_delta_hz（大 Δf 下角速度欠采样、换算失效）。
            _fill = float(measurement.get("fill_ratio", 0.0))
            if (
                _fill > RAMP_SWEEP_FILL_TRIGGER
                and self.local_sweep_count < RAMP_SWEEP_MAX_ATTEMPTS
            ):
                return self._begin_local_sweep(now_ms, _fill)
            angle = measurement.get("principal_axis_angle_deg")
            if angle is None:
                return None
            self.rotation_samples.append((float(angle), int(now_ms)))
            if len(self.rotation_samples) < ROTATION_SAMPLES:
                return None
            deg_per_sec = self._rotation_slope_deg_per_sec()
            # D4 修复：2:1 目标（∞ 形）旋转速率与 Δf 比例与 1:1 不同，
            # 用独立标定系数；1:1 沿用 ROTATION_HZ_PER_TURN_DEG。
            hz_per_turn = (
                ROTATION_HZ_PER_TURN_DEG_2TO1
                if self.target_shape == 3
                else ROTATION_HZ_PER_TURN_DEG
            )
            delta_hz = float(deg_per_sec) / hz_per_turn
            # D10 修复：Δf 小且时间窗不足时角度噪声主导（5 帧≈160ms），
            # 继续收集样本延长观测窗（最多 ROTATION_MAX_SAMPLES 帧兜底）。
            window_ms = ticks_diff(
                self.rotation_samples[-1][1], self.rotation_samples[0][1]
            )
            if (
                abs(delta_hz) < ROTATION_MIN_DELTA_HZ
                and window_ms < ROTATION_MIN_WINDOW_MS
                and len(self.rotation_samples) < ROTATION_MAX_SAMPLES
            ):
                return None
            # D4 修复：方向符号由 config rotation_sign 决定（默认 +1，
            # 保持"主轴正向旋转(角度递增)=fy>fx -> 减频"原语义；标定发现
            # 方向反了设 -1，否则锁频会发散）。
            # D13 修复(2026-10-22)：输出频率必须严格落在 100Hz 网格。
            # 赛题范围 1kHz~100kHz、步进 100Hz，只有 (100000-1000)/100+1
            # = 991 个合法频率点，不存在比 100Hz 更精确的值。旋转角速度
            # 换算的 Δf 是连续值，修正后若直接下发会得到 49.999kHz 这类
            # 网格外频率（F407 按 mHz 全精度执行），必须先归整到 100Hz。
            _corrected = (
                self.locked_frequency_millihz
                - ROTATION_SIGN * round(delta_hz * 1000.0)
            )
            _corrected = int(round(_corrected / 100000.0)) * 100000
            self.locked_frequency_millihz = int(clamp(
                _corrected,
                SWEEP_GRID_MIN * 100000,
                self._output_grid_max() * 100000,
            ))
            print("[锁频] 角速度=%.1f deg/s 频率差=%.2f Hz 校正后输出=%.3f kHz（100Hz 网格）" % (
                deg_per_sec, delta_hz, self.locked_frequency_millihz / 1.0e6))
            self.rotation_samples = []
            self.rotation_rounds += 1
            self.state = self.PHASE_FINE
            # 幅度预收敛：先发一帧 phase=0，用下一帧的 span_y 检查器件
            # Y 幅度。span_y 只取决于输出幅度，与相对相位无关，因此可以
            # 在相位扫描之前独立收敛，避免"幅度偏差 -> 形状永不 pass ->
            # 进不了 AMPLITUDE_FINE"的死锁。
            self.phase_amplitude_checked = False
            self.amplitude_corrections = 0
            self.phase_good_frames = 0
            return self._issue_millihz(
                self.locked_frequency_millihz, 0.0, self.amplitude_mv,
                self._next_control_probe_id(), now_ms
            )

        if self.state == self.PHASE_FINE:
            if self.phase_scan_active:
                return self._phase_scan_step(measurement, now_ms)
            if not self.phase_amplitude_checked:
                return self._phase_amplitude_precheck(measurement, now_ms)
            # v2：相位爬山由真机标定增益驱动，此处以"目标形状连续 pass"
            # 作为收敛判据（旋转锁频已把 Δf 压到主轴不可分辨，残余相位
            # 误差由 HOLD 漂移检测兜底）。连续 pass 帧数到位即进入幅度闭环。
            if target_shape_ok(measurement, self.target_shape):
                self.phase_good_frames += 1
                if self.phase_good_frames >= PHASE_FINE_GOOD_FRAMES:
                    print("[相位] 目标形状连续通过 %d 帧 -> 进入幅度闭环" % (
                        self.phase_good_frames))
                    self.state = self.AMPLITUDE_FINE
                    self.amplitude_samples = []
                    self.amplitude_corrections = 0
                    self.amplitude_invalid_frames = 0
                    return self._issue_millihz(
                        self.locked_frequency_millihz, self.phase_deg,
                        self.amplitude_mv,
                        self._next_control_probe_id(), now_ms
                    )
                return None
            self.phase_good_frames = 0
            return None

        if self.state == self.AMPLITUDE_FINE:
            # v2：幅度闭环判据 = 目标形状 pass + Y span 有效（不依赖计数比）。
            if (
                not target_shape_ok(measurement, self.target_shape)
                or float(measurement.get("span_y_div", 0.0)) <= 0.0
            ):
                self.amplitude_invalid_frames += 1
                if self.amplitude_invalid_frames >= 6:
                    return self._fail(
                        self.ERROR_AMPLITUDE,
                        "shape was invalid during amplitude adjustment",
                    )
                return None
            self.amplitude_invalid_frames = 0
            self.amplitude_samples.append(
                float(measurement.get("span_y_div", 0.0))
            )
            if len(self.amplitude_samples) < 3:
                return None
            span_y = float(median(self.amplitude_samples))
            self.amplitude_samples = []
            error = self.target_span_div - span_y
            if abs(error) <= AMPLITUDE_CONVERGE_DIV:
                # 幅度收敛后上报锁定状态；完成提示延迟到稳定保持期结束。
                print("[幅度] span_y=%.2f div 目标=%.2f div 误差=%+.2f -> LOCKED，开始稳定保持" % (
                    span_y, self.target_span_div, error))
                self.state = self.HOLD
                self.hold_since_ms = int(now_ms)
                self.hard_fail_frames = 0
                self.hold_fill_reference = float(
                    measurement.get("fill_ratio", 0.0)
                )
                self.hold_geometry_reference = (
                    float(measurement.get('span_x_div', 0.0)),
                    float(measurement.get('span_y_div', 0.0)),
                    float(measurement.get('center_x_div', 0.0)),
                    float(measurement.get('center_y_div', 0.0)),
                )
                self.last_hold_sample_ms = int(now_ms)
                return LockedEvent(
                    self.sequence, self._reported_input_frequency_millihz()
                )
            if self.amplitude_corrections >= 4:
                return self._fail(
                    self.ERROR_AMPLITUDE,
                    "Y amplitude did not converge in four corrections",
                )
            self._adjust_amplitude_for_span(span_y)
            self.amplitude_corrections += 1
            return self._issue_millihz(
                self.locked_frequency_millihz, self.phase_deg, self.amplitude_mv,
                self._next_control_probe_id(), now_ms
            )

        if self.state == self.HOLD:
            return self._hold_step(measurement, now_ms)

        if self.state == self.DONE:
            # DONE 是事件：任务完成，不再下发指令。
            return None
        return None

    def _control_timeout(self, now_ms):
        if (
            self.started_ms is not None
            and self.state not in (
                self.HOLD, self.DONE, self.FAILED, self.WAIT_START
            )
            and ticks_diff(now_ms, self.started_ms) >= self.control_timeout_ms
        ):
            return self._fail(
                self.ERROR_CONTROL_TIMEOUT,
                "LOCKED was not reached before the 10 s deadline",
            )
        return None

    def tick(self, now_ms):
        if self.state in (self.WAIT_START, self.FAILED):
            return None
        timeout = self._control_timeout(now_ms)
        if timeout is not None:
            return timeout
        if (
            not self.waiting_for_applied or self.last_command is None
            or self.ack_started_ms is None
            or ticks_diff(now_ms, self.ack_started_ms) < self.ack_timeout_ms
        ):
            return None
        if self.ack_retries < self.max_ack_retries:
            self.ack_retries += 1
            self.ack_started_ms = int(now_ms)
            return self.last_command
        return self._fail(
            self.ERROR_UART_TIMEOUT, "APPLIED acknowledgement timed out"
        )

    def _fail(self, error_code, reason):
        self.state = self.FAILED
        self.failure_code = int(error_code)
        self.failure_reason = str(reason)
        self.waiting_for_applied = False
        print("[失败] code=%d 阶段=%s 原因: %s" % (
            int(error_code), self._state_name_cn(), reason))
        return FailedEvent(self.sequence, error_code, reason)

    def _state_name_cn(self):
        names = {
            self.WAIT_START: "等待启动",
            self.MEASURE_X_AMPLITUDE: "测量X幅度",
            self.RAMP_ESTIMATE: "斜坡粗测频",
            self.ROTATION_LOCK: "旋转锁频",
            self.PHASE_FINE: "相位微调",
            self.AMPLITUDE_FINE: "幅度闭环",
            self.HOLD: "稳定5秒",
            self.DONE: "完成",
            self.FAILED: "失败",
        }
        return names.get(self.state, str(self.state))

    def stop(self):
        saved_sequence = self.sequence
        self._reset_cycle()
        self.sequence = saved_sequence

    def hold_elapsed_ms(self, now_ms):
        if self.state not in (self.HOLD, self.DONE):
            return 0
        if self.hold_since_ms is None:
            return 0
        return max(0, ticks_diff(now_ms, self.hold_since_ms))

# 早期 K230 主机测试使用的向后兼容名称。
FastSearchController = Requirement5Controller

def analyze_frame(img, roi):
    return analyze_frame_for_stage(img, roi)

def analyze_frame_for_stage(img, roi, mode="full", target_shape=None):
    started=ticks_ms()
    # v3：斜坡展开帧——行质心 + 整数周期 DFT + LSQ 拟合估 N=fx·T_ramp。
    # 轨迹是 x=x(y) 单值曲线（Y=斜坡相位轴），细采样保相位信息；
    # 输出 ramp_cycles/ramp_quality 供 RAMP_ESTIMATE 状态机消费。
    if mode == "ramp":
        points = collect_yellow_points(img, roi, step=TRACE_SAMPLE_STEP)
        extracted = ticks_ms()
        result = empty_measurement("RAMP_NO_TRACE")
        n_cycles, quality = estimate_ramp_cycles(points, roi)
        result["ramp_cycles"] = n_cycles
        result["ramp_quality"] = quality
        result["trace_pixels"] = len(points)
        result["extract_ms"] = ticks_diff(extracted, started)
        result["analysis_ms"] = ticks_diff(ticks_ms(), extracted)
        return result
    # FPS-OPT: counts/span 模式粗采样（step=6 -> 采样点约 1/4），
    # full 模式细采样（step=3）。实测 SEARCH 满格包络 analysis_ms 120ms+
    # -> 粗采样后约 30-40ms，10s 预算内能跑完 41 档搜索。
    # FPS-OPT2: counts/span 先做一次 C 层 fast 预判——满格旋转包络
    # （fill_ratio 高，远离小整数比）直接返回快速帧，跳过 Python 遍历与
    # 四边投影；只有 fill 低的静态候选才进完整 counts。满格档从 ~170ms
    # 压到一次 find_blobs（~15ms），SEARCH 每档开销主要由 SET 等待决定。
    if mode in ("counts", "span"):
        fast = fast_trace_stats(img, roi)
        if fast is None:
            result = make_fast_measurement(None, roi)
            result["extract_ms"] = ticks_diff(ticks_ms(), started)
            result["analysis_ms"] = 0.0
            return result
        if mode == "span":
            result = make_fast_measurement(fast, roi)
            result["extract_ms"] = ticks_diff(ticks_ms(), started)
            result["analysis_ms"] = 0.0
            return result
        if fast["fill_ratio"] > FAST_FILL_THRESHOLD:
            result = make_fast_measurement(fast, roi)
            result["valid"] = False
            result["strict_pair_valid"] = False
            result["invalid_reason"] = "SEARCH_FILL_GATE"
            result["extract_ms"] = ticks_diff(ticks_ms(), started)
            result["analysis_ms"] = 0.0
            return result
    sample_step = 6 if mode in ("counts", "span") else TRACE_SAMPLE_STEP
    points=collect_yellow_points(img, roi, step=sample_step)
    extracted=ticks_ms()
    result=analyze_trace_points(points,roi[2],roi[3],mode,target_shape,
                                step=sample_step)
    result['extract_ms']=ticks_diff(extracted,started)
    result['analysis_ms']=ticks_diff(ticks_ms(),extracted)
    contacts_local=result.pop("contacts_local", [])
    # 单边可能看起来合理，但整体测量失败。
    # 此类候选不能作为被接受的波形触点显示。
    result["contacts"]=[(int(x+roi[0]),int(y+roi[1]))
                        for x,y in contacts_local] if result["valid"] else []
    return result

class UartFrameParser:
    def __init__(self):
        self.collecting = False
        self.buffer = ""

    def feed(self, data):
        if not data:
            return []
        if isinstance(data, bytes):
            # MicroPython 的 bytes.decode('ascii','ignore') 在部分固件上遇到非法
            # 字节仍会抛 UnicodeError 导致整段崩溃。改为手动只保留可打印 ASCII
            # 与控制字符(\t \n \r)，其余字节丢弃，解析永不抛异常。协议帧本身
            # 是纯 ASCII($START,...,\r\n)，过滤杂字节不影响帧解析。
            data = "".join(chr(b) for b in data if 32 <= b < 127 or b in (9, 10, 13))
        frames = []
        for character in str(data):
            if character == "$":
                self.collecting = True
                self.buffer = ""
            elif character == "\r" or character == "\n":
                if self.collecting and self.buffer:
                    frames.append(self.buffer)
                self.collecting = False
                self.buffer = ""
            elif self.collecting:
                if len(self.buffer) < 127:
                    self.buffer += character
                else:
                    self.collecting = False
                    self.buffer = ""
        return frames

def parse_uart_frame(frame):
    """START/APPLIED/ERROR 返回元组，否则返回 None。"""
    fields = str(frame).split(",")
    try:
        # $START,shape[,prior_mhz] —— prior 只是台架提示。
        if len(fields) in (2, 3) and fields[0] == "START":
            shape = int(fields[1])
            if shape in (1, 2, 3):
                prior = int(fields[2]) if len(fields) == 3 else 0
                return ("START", shape, prior)
        elif len(fields) == 3 and fields[0] == "APPLIED":
            return ("APPLIED", int(fields[1]), int(fields[2]))
        elif len(fields) == 4 and fields[0] == "ERROR":
            return (
                "ERROR", int(fields[1]), int(fields[2]), int(fields[3])
            )
    except (TypeError, ValueError):
        return None
    return None

def format_action(action):
    if isinstance(action, dict) and action.get("kind") == "RAMP":
        # v3：$RAMP,seq,probe,saw_hz,amp_mv —— F407 切 PA4 DAC 输出锯齿斜坡
        return "$RAMP,%d,%d,%d,%d\r\n" % (
            action["sequence"], action["probe_id"],
            action["saw_hz"], action["amplitude_mv"],
        )
    if isinstance(action, dict) and action.get("kind") == "SET":
        return "$SET,%d,%d,%d,%d,%d,%d\r\n" % (
            action["sequence"], action["probe_id"],
            action["frequency_millihz"], action["phase_millideg"],
            action["amplitude_mv"], action["shape"],
        )
    if isinstance(action, DoneEvent):
        return "$DONE,%d,%d\r\n" % (
            action.sequence, action.input_frequency_millihz
        )
    if isinstance(action, LockedEvent):
        return "$LOCKED,%d,%d\r\n" % (
            action.sequence, action.input_frequency_millihz
        )
    if isinstance(action, FailedEvent):
        return "$FAILED,%d,%d\r\n" % (
            action.sequence, action.error_code
        )
    return None

def send_action(uart, action):
    message = format_action(action)
    if message is None:
        return False
    uart.write(message)
    # 诊断日志：任何发往 F407 的帧都会在 CanMV IDE 终端显示
    if UART_VERBOSE or isinstance(action, (LockedEvent, DoneEvent, FailedEvent)):
        print("[串口TX] >> %s" % message.strip())
    if isinstance(action, FailedEvent):
        print("[控制失败] %s" % action.reason)
    return isinstance(action, dict)

def consume_uart(uart, parser, controller, now_ms):
    """返回 (actions, matching_applied_seen)。"""
    actions = []
    applied_seen = False
    try:
        pending = uart.any()
    except Exception:
        pending = 0
    if not pending:
        return actions, applied_seen
    data = uart.read(pending)
    for raw_frame in parser.feed(data):
        frame = parse_uart_frame(raw_frame)
        if frame is None:
            print("[串口] 忽略未知帧: %s" % raw_frame)
            continue
        if frame[0] == "START":
            # 重发的 START 不能重启正在运行的进程。不同的 shape，
            # 或 DONE/FAILED 之后的 START，仍会开启新周期。
            duplicate = bool(
                controller.target_shape == frame[1]
                and controller.started_ms is not None
                and controller.state not in (
                    controller.WAIT_START, controller.DONE,
                    controller.FAILED,
                )
            )
            # 诊断日志：收到 F407 的开始命令（3 字段格式；freq=装置自身输出频率）
            if UART_VERBOSE or not duplicate:
                print("[串口RX] << START 模式=%d 当前输出=%.3f kHz 重复=%d" % (
                    frame[1], frame[2] / 1.0e6, duplicate))
            if not duplicate:
                # 3 字段协议 $START,shape,freq_millihz —— freq 是装置自身
                # 当前输出频率（DDS 内部状态，非信号源频率），仅作上下文记录；
                # 题目要求5 断耦合、纯摄像头、频率点不得传入/预设，因此不得
                # 携带信号源频率；测频一律走 RAMP 斜坡粗测频，与 freq 无关。
                actions.append(
                    controller.start(frame[1], frame[2], now_ms)
                )
        elif frame[0] == "APPLIED":
            # 诊断日志：F407 已执行 SET 并应答
            if UART_VERBOSE:
                print("[串口RX] << APPLIED 已执行 序号=%d 探针=%d" % (frame[1], frame[2]))
            if controller.on_applied(frame[1], frame[2], now_ms):
                applied_seen = True
        elif frame[0] == "ERROR":
            print("[串口RX] << ERROR F407报错 序号=%d 探针=%d 错误码=%d" % (
                frame[1], frame[2], frame[3]))
            action = controller.on_error(frame[1], frame[2], frame[3])
            if action is not None:
                actions.append(action)
    return actions, applied_seen

def _sleep_ms(milliseconds):
    try:
        time.sleep_ms(int(milliseconds))
    except AttributeError:
        time.sleep(float(milliseconds) / 1000.0)

def service_uart(uart, parser, controller):
    """先排空回复再检查时限，包括慢视觉工作之后。"""
    if uart is None or parser is None or controller is None:
        return
    now_ms = ticks_ms()
    actions, unused = consume_uart(uart, parser, controller, now_ms)
    for action in actions:
        send_action(uart, action)
    timed = controller.tick(now_ms)
    if timed is not None:
        send_action(uart, timed)

def init_uart2():
    fpioa = FPIOA()
    fpioa.set_function(UART2_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(UART2_RX_PIN, FPIOA.UART2_RXD)
    return UART(
        UART.UART2,
        baudrate=UART2_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )

try:
    with open("/sdcard/grid_yolo/config.json") as f:
        CONFIG = ujson.loads(f.read())
    print("[配置] 已加载 /sdcard/grid_yolo/config.json")
except (OSError, ValueError, TypeError):
    # 真机上必有 config.json；PC 回归测试/异常启动时全部走默认值。
    CONFIG = {}
# v3：RAMP 斜坡测频可配置覆盖（/sdcard/grid_yolo/config.json）
RAMP_ENABLED = bool(CONFIG.get("ramp_enabled", True))
RAMP_SAW_HZ = int(CONFIG.get("ramp_saw_hz", 1000))
RAMP_AMP_MV = int(CONFIG.get("ramp_amp_mv", 3300))
RAMP_MIN_CYCLES = int(CONFIG.get("ramp_min_cycles", 1))
RAMP_MAX_CYCLES = int(CONFIG.get("ramp_max_cycles", 60))
# D1 修复：质量门限与泄漏嫌疑周期数也做成可配（真机噪声大时可下调门限）
RAMP_QUALITY_GATE = float(CONFIG.get("ramp_quality_gate", 0.50))
RAMP_SUSPECT_CYCLES = int(CONFIG.get("ramp_suspect_cycles", 55))
RAMP_CONFIRM_CYCLES = int(CONFIG.get("ramp_confirm_cycles", 20))
# D14：斜坡满幅高度与估频中值帧数可配（真机 DAC 电压/噪声差异大时可校准）
RAMP_FULL_HEIGHT_DIV = float(CONFIG.get("ramp_full_height_div", 6.6))
RAMP_MEDIAN_FRAMES = max(5, int(CONFIG.get("ramp_median_frames", 7)))
RAMP_MAX_RELATIVE_SPREAD = float(CONFIG.get(
    "ramp_max_relative_spread", RAMP_MAX_RELATIVE_SPREAD))
RAMP_MAX_RELATIVE_MAD = float(CONFIG.get(
    "ramp_max_relative_mad", RAMP_MAX_RELATIVE_MAD))
RAMP_UNSTABLE_SAW_HZ = int(CONFIG.get(
    "ramp_unstable_saw_hz", RAMP_UNSTABLE_SAW_HZ))
RAMP_DISCARD_FRAMES_AFTER_APPLIED = max(0, int(CONFIG.get(
    "ramp_discard_frames_after_applied", RAMP_DISCARD_FRAMES_AFTER_APPLIED)))
RAMP_LOCAL_SWEEP_RANGE_GRID = int(CONFIG.get(
    "ramp_local_sweep_range_grid", 12))
# v3.1：局部网格扫描判据参数（config.json 可现场覆盖）
RAMP_SWEEP_FILL_TRIGGER = float(CONFIG.get(
    "ramp_sweep_fill_trigger", 0.35))
RAMP_SWEEP_STATIC_FILL = float(CONFIG.get(
    "ramp_sweep_static_fill", 0.30))
RAMP_SWEEP_FILL_MARGIN = float(CONFIG.get(
    "ramp_sweep_fill_margin", 0.08))
RAMP_SWEEP_FRAMES = max(1, int(CONFIG.get("ramp_sweep_frames", 2)))
RAMP_SWEEP_MAX_ATTEMPTS = max(1, int(CONFIG.get(
    "ramp_sweep_max_attempts", 2)))
KNOWN_Y_HZ=CONFIG.get("known_y_hz")
GRID_DIVISIONS_X=float(CONFIG.get("grid_divisions_x",8))
GRID_DIVISIONS_Y=float(CONFIG.get("grid_divisions_y",8))
FRAME_ANALYZE_INTERVAL=max(1,int(CONFIG.get("idle_analyze_interval",3)))
MASK_BUFFER_ENABLED=bool(CONFIG.get("mask_buffer_enabled",True))
# D4 修复：旋转方向符号与 2:1 系数真机标定项（config.json 覆盖）
ROTATION_SIGN = int(CONFIG.get("rotation_sign", 1))
ROTATION_HZ_PER_TURN_DEG = float(CONFIG.get("rotation_hz_per_turn_deg", 180.0))
ROTATION_HZ_PER_TURN_DEG_2TO1 = float(CONFIG.get(
    "rotation_hz_per_turn_deg_2to1", 180.0))
# D9 修复：示波器 Y 档位（现场锁定该值，默认 0.5V/div）
VOLTS_PER_DIV = float(CONFIG.get("volts_per_div", 0.5))
# 2026-10-22：可选的手动曝光/白平衡锁定。默认 0/False 保持原自动行为。
# 自动曝光在轨迹亮度变化时会来回漂移，导致 YELLOW_THRESHOLD 检出量
# 与 span 抖动；真机调好亮度后建议在 config.json 写 manual_exposure_us。
MANUAL_EXPOSURE_US = int(CONFIG.get("manual_exposure_us", 0))
LOCK_WHITE_BALANCE = bool(CONFIG.get("lock_white_balance", False))
# D11 修复：摄像头帧率（按 sensor 型号与带宽实测调）
SENSOR_FPS = int(CONFIG.get("sensor_fps", 30))
# FPS-OPT: 网格锁定后 10 -> 20 帧复查一次（固定机位网格几乎不动），
# 每次复查含一次 640x480->320x320 ai2d 缩放 + KPU 推理。
GRID_RECHECK_FRAMES=max(1,int(CONFIG.get("grid_recheck_frames",20)))
# FPS-OPT: 锁定前每帧跑 YOLO 只为确认网格；隔 2 帧推理一次即可，
# 把开机阶段的推理开销砍一半（CONFIRM_FRAMES=8 -> 确认耗时减半）。
GRID_REINFER_EVERY_FRAMES=max(1,int(CONFIG.get("grid_reinfer_every_frames",2)))
# FPS-OPT: 显示降频。1=每帧显示；2=隔帧显示（显示 15fps 仍流畅，
# CPU 上的显示拷贝/缩放开销减半；比赛现场可接受）。
DISPLAY_EVERY_N_FRAMES=max(1,int(CONFIG.get("display_every_n_frames",2)))
# FPS-OPT: YOLO 推理通道分辨率。训练预处理是 640x480 -> resize 0.5
# -> 320x240 -> 上下各 pad 40px -> 320x320。推理通道直接输出 320x240
# 与训练缩放后画面语义一致，decode_grid 的 SCALE=0.5/PAD_Y=40 不用改；
# sensor 双通道带宽与 ai2d 输入数据量降为 1/4（实测 capture_ms 13-27ms，
# 是第二瓶颈）。GC2093/OV5647 均支持 320x240；若固件报错，config.json
# 设 "yolo_chn_w": 640, "yolo_chn_h": 480 回退。
YOLO_CHN_W=int(CONFIG.get("yolo_chn_w",320))
YOLO_CHN_H=int(CONFIG.get("yolo_chn_h",240))
POST_APPLIED_SETTLE_MS=max(0,int(CONFIG.get("post_applied_settle_ms",100)))
UART_VERBOSE=bool(CONFIG.get("uart_verbose",False))
MEASUREMENT_LOG_MS=max(200,int(CONFIG.get("measurement_log_ms",1000)))
if CANMV_AVAILABLE:
    from media.sensor import Sensor
    from media.display import Display
    from media.media import MediaManager
else:
    Sensor = None
    Display = None
    MediaManager = None

MODEL_PATH = '/sdcard/grid_yolo/best.kmodel'
CONFIDENCE = 0.50
ROTATE_IMAGE_180 = True
DISPLAY_TARGET = 'lcd'   # lcd（ST7701）或 virt（仅 IDE）
# FPS-OPT: 默认关闭 IDE 预览。to_ide=True 时每帧还要经 USB 把画面推给
# CanMV IDE，双通道 640x480 下会挤占 MIPI/USB 带宽；关闭后帧率更稳。
# 需要 IDE 看画面时在 config.json 设 "preview_to_ide": true。
PREVIEW_TO_IDE=bool(CONFIG.get("preview_to_ide",False))
SAVE_LOCKED_CROP = False
CONFIRM_FRAMES = 8
ROI_TOLERANCE_PX = 6
WIDTH, HEIGHT = 640, 480
INPUT_SIZE = 320
# 训练预处理：640x480 -> 320x240 + 上下各 40px 填充。
SCALE = 0.5
PAD_X, PAD_Y = 0, 40

_GRID_SHAPE_WARNED = False

def decode_grid(output):
    # YOLO11n 单类检测输出：[1,5,2100]，cx/cy/w/h/confidence。
    # 2026-10-22：形状不符（模型/固件不匹配）时返回“本帧无检测”而不是
    # 抛异常——旧实现会让异常逃到 main() 顶层直接终止整个程序。
    if len(output.shape) != 3 or output.shape[1] != 5:
        global _GRID_SHAPE_WARNED
        if not _GRID_SHAPE_WARNED:
            _GRID_SHAPE_WARNED = True
            print('[网格] 模型输出形状异常 %s，按无检测处理' % str(output.shape))
        return None, 0.0
    values = output[0]
    index = int(np.argmax(values[4]))
    score = float(values[4][index])
    if score < CONFIDENCE:
        return None, score
    cx,cy,w,h = [float(values[i][index]) for i in range(4)]
    x1 = max(0, min(WIDTH-1, int(round((cx-w/2-PAD_X)/SCALE))))
    y1 = max(0, min(HEIGHT-1, int(round((cy-h/2-PAD_Y)/SCALE))))
    x2 = max(0, min(WIDTH, int(round((cx+w/2-PAD_X)/SCALE))))
    y2 = max(0, min(HEIGHT, int(round((cy+h/2-PAD_Y)/SCALE))))
    if x2-x1 < 40 or y2-y1 < 40:
        return None, score
    # 本应用期望恰好一个网格；取最高分。
    return (x1,y1,x2-x1,y2-y1), score

def close_roi(a,b):
    return a is not None and b is not None and all(abs(x-y)<=ROI_TOLERANCE_PX for x,y in zip(a,b))

# ---------------------------------------------------------------------------
# 固定背景差分：差分 AND 黄色（去掉静止的黄色光标/网格/UI，只留运动轨迹）
# 配置键(从 /sdcard/grid_yolo/config.json 读取)：
#   background_enabled    是否启用差分（默认 false，回退纯颜色检测）
#   background_threshold  差异门限（默认 25）
#   capture_background    本次运行先采集空背景（需隐藏波形，网格+光标保留）
#   background_frames     采集平均帧数（默认 8）
# ---------------------------------------------------------------------------
BG_PATH='/sdcard/grid_yolo/trace_background.bin'
BG_META='/sdcard/grid_yolo/trace_background.json'
BG_ENABLED=bool(CONFIG.get('background_enabled',False))
BG_THRESHOLD=int(CONFIG.get('background_threshold',25))
CAPTURE_BACKGROUND=bool(CONFIG.get('capture_background',CONFIG.get('background_capture_on_start',False)))
BG_FRAMES=int(CONFIG.get('background_frames',8))
BG_ROI=None
BG_SAMPLES=[]
BACKGROUND=None
BG_SUM=None
BG_COUNT=0
BG_CAPTURING=False
BG_WARNED=False
BG_STATUS='READY_COLOR_ONLY'

def _rgb_from_pixel(pixel):
    if isinstance(pixel,(tuple,list)):
        return (int(pixel[0]),int(pixel[1]),int(pixel[2]))
    value=int(pixel)
    return (((value>>11)&31)*255//31,((value>>5)&63)*255//63,(value&31)*255//31)

def _pixel_differs(rgb,ref,threshold):
    return max(abs(int(rgb[i])-int(ref[i])) for i in range(3))>=int(threshold)

def _pixel_differs_at(rgb, data, offset, threshold):
    # 避免为每个黄色样本分配 bytes 切片与生成器。
    return (abs(rgb[0]-data[offset])>=threshold
            or abs(rgb[1]-data[offset+1])>=threshold
            or abs(rgb[2]-data[offset+2])>=threshold)

def build_bg_sampling(roi):
    rx,ry,rw,rh=roi
    samples=[]
    for ly in range(0,rh,TRACE_SAMPLE_STEP):
        for lx in range(0,rw,TRACE_SAMPLE_STEP):
            samples.append((lx,ly))
    return samples

def bg_meta_dict(roi):
    return {'image_size':[WIDTH,HEIGHT],'rotate_180':ROTATE_IMAGE_180,
            'roi':[int(v) for v in roi],'step':TRACE_SAMPLE_STEP,
            'count':len(build_bg_sampling(roi))}

def load_background():
    global BACKGROUND,BG_ROI,BG_SAMPLES,BG_STATUS
    BACKGROUND=None
    try:
        with open(BG_META) as f:
            meta=ujson.loads(f.read())
        if meta.get('image_size')!=[WIDTH,HEIGHT] or meta.get('rotate_180')!=ROTATE_IMAGE_180:
            BG_STATUS='BACKGROUND_CALIBRATION_MISMATCH'
            return BG_STATUS
        roi=tuple(int(v) for v in meta['roi'])
        if (meta.get('step')!=TRACE_SAMPLE_STEP or len(roi)!=4
                or roi[0]<0 or roi[1]<0 or roi[2]<=0 or roi[3]<=0
                or roi[0]+roi[2]>WIDTH or roi[1]+roi[3]>HEIGHT):
            BG_STATUS='BACKGROUND_CALIBRATION_MISMATCH'
            return BG_STATUS
        samples=build_bg_sampling(roi)
        if meta.get('count')!=len(samples):
            BG_STATUS='BACKGROUND_SIZE_ERROR'
            return BG_STATUS
        with open(BG_PATH,'rb') as f:
            data=f.read()
        if len(data)!=len(samples)*3:
            BG_STATUS='BACKGROUND_SIZE_ERROR'
            return BG_STATUS
        BACKGROUND=data;BG_ROI=roi;BG_SAMPLES=samples
        BG_STATUS='READY'
        print('[背景] 已加载 ROI=%s 样本=%d 字节=%d' % (str(roi),len(samples),len(data)))
        return BG_STATUS
    except (OSError, ValueError, KeyError, TypeError):
        BG_STATUS='NEED_EMPTY_BACKGROUND'
        return BG_STATUS

def begin_bg_capture(roi):
    global BG_ROI,BG_SAMPLES,BG_SUM,BG_COUNT,BG_CAPTURING,BACKGROUND,BG_STATUS
    BG_ROI=tuple(roi)
    BG_SAMPLES=build_bg_sampling(BG_ROI)
    BG_SUM=bytearray(len(BG_SAMPLES)*6)
    BG_COUNT=0
    BG_CAPTURING=True
    BACKGROUND=None
    BG_STATUS='HIDE_WAVEFORM_CAPTURE_BACKGROUND'
    print('[背景] 开始采集背景：请保持屏幕空白（仅网格+光标、无波形）样本数=%d' % len(BG_SAMPLES))

def accumulate_bg(img):
    global BG_COUNT,BG_CAPTURING,BG_SUM,BACKGROUND,BG_ENABLED,BG_ROI,BG_SAMPLES,BG_STATUS
    if not BG_CAPTURING or BG_SUM is None or BG_ROI is None:
        return False
    rx,ry=BG_ROI[0],BG_ROI[1]
    for i,(lx,ly) in enumerate(BG_SAMPLES):
        rgb=_rgb_from_pixel(img.get_pixel(rx+lx,ry+ly))
        for c in range(3):
            k=(i*3+c)*2
            value=BG_SUM[k]+(BG_SUM[k+1]<<8)+rgb[c]
            BG_SUM[k]=value&255;BG_SUM[k+1]=value>>8
    BG_COUNT+=1
    target=max(2,min(32,BG_FRAMES))
    if BG_COUNT<target:
        return False
    data=bytearray(len(BG_SAMPLES)*3)
    for i in range(len(data)):
        data[i]=(BG_SUM[i*2]+(BG_SUM[i*2+1]<<8)+BG_COUNT//2)//BG_COUNT
    try:
        with open(BG_PATH+'.tmp','wb') as f:
            f.write(data)
        with open(BG_META+'.tmp','w') as f:
            f.write(ujson.dumps(bg_meta_dict(BG_ROI)))
        for name in (BG_META,BG_PATH):
            try:os.remove(name)
            except OSError:pass
        os.rename(BG_PATH+'.tmp',BG_PATH)
        os.rename(BG_META+'.tmp',BG_META)
    except Exception as error:
        print('[背景] 背景保存失败:',error)
        BG_CAPTURING=False;BG_SUM=None;BG_STATUS='BACKGROUND_SAVE_FAILED'
        return True
    BACKGROUND=data;BG_SUM=None;BG_CAPTURING=False;BG_ENABLED=True
    BG_STATUS='READY'
    print('[背景] 背景已保存 帧数=%d 字节=%d' % (BG_COUNT,len(data)))
    return True

def binary_mask_buffer(mask, width, height):
    """读取未打包的 RGB565 黑白 mask，不做逐像素调用。

    未知、填充或打包布局保留 get_pixel 回退。
    """
    try:
        view = memoryview(mask.to_numpy_ref())
        size = width * height
        itemsize = view.itemsize
        if len(view) == size and itemsize == 2:
            return view, 1
        if len(view) == size * 2 and itemsize == 1:
            # RGB565 black=0x0000, white=0xffff：任一个字节都足以判断。
            return view, 2
    except (AttributeError, TypeError, ValueError, NotImplementedError):
        pass
    return None, 0

def collect_diff_points(img,roi):
    """差分 AND 黄色：只扫描黄色连通域并集，步长不变、测量精度不变，排除静止光标。

    只走所有黄色 blob 的包围盒并集（而非整个 ROI），把每帧采样点从约 1.6 万个
    降到黄色区域规模，降低 K230 帧率压力。光标是黄色块、轨迹是黄色块，并集覆盖
    两者，再由“黄色 AND 与空背景差异>=门限”把静止光标滤掉，只留运动轨迹。
    """
    global MASK_ACCESS_MODE
    rx,ry,rw,rh=roi
    threshold=int(BG_THRESHOLD)
    step=TRACE_SAMPLE_STEP
    try:
        blobs=img.find_blobs([YELLOW_THRESHOLD],roi=roi,pixels_threshold=FAST_BLOB_PIXELS_THRESHOLD,merge=True)
    except Exception as error:
        print("[差分] collect_diff_points find_blobs 异常:",error)
        blobs=[]
    if not blobs:
        return []
    # 所有黄色 blob 的包围盒并集（转 ROI 局部坐标）
    min_lx=rw; min_ly=rh; max_lx=0; max_ly=0
    for blob in blobs:
        try: x,y,w,h=blob.rect()
        except Exception: continue
        x-=rx; y-=ry
        if x<min_lx: min_lx=x
        if y<min_ly: min_ly=y
        if x+w>max_lx: max_lx=x+w
        if y+h>max_ly: max_ly=y+h
    if max_lx<=min_lx or max_ly<=min_ly:
        return []
    # 外扩 2px 容忍；扫描坐标对齐到 step 网格（与 build_bg_sampling 采样点一致）
    min_lx=max(0,min_lx-2); min_ly=max(0,min_ly-2)
    max_lx=min(rw,max_lx+2); max_ly=min(rh,max_ly+2)
    start_lx=((min_lx+step-1)//step)*step
    start_ly=((min_ly+step-1)//step)*step
    mask=img.copy(roi=roi)
    mask.binary([YELLOW_THRESHOLD])
    mask_view,mask_stride=binary_mask_buffer(mask,rw,rh) if MASK_BUFFER_ENABLED else (None,0)
    MASK_ACCESS_MODE='buffer' if mask_view is not None else 'get_pixel'
    mask_get=mask.get_pixel
    image_get=img.get_pixel
    is_white=pixel_is_white
    to_rgb=_rgb_from_pixel
    differs=_pixel_differs_at
    background=BACKGROUND
    stride=(rw+step-1)//step
    points=[]
    append=points.append
    for ly in range(start_ly,max_ly,step):
        row_base=(ly//step)*stride
        mask_row=ly*rw
        for lx in range(start_lx,max_lx,step):
            if mask_view is not None:
                if not mask_view[(mask_row+lx)*mask_stride]:
                    continue
            elif not is_white(mask_get(lx,ly)):
                continue
            idx=row_base+(lx//step)
            rgb=to_rgb(image_get(rx+lx,ry+ly))
            if differs(rgb,background,idx*3,threshold):
                append((lx,ly))
    mask_view=None
    mask=None
    return points

def save_calibration(img,roi):
    parent='/sdcard/grid_yolo'
    for index in range(1,10000):
        folder=parent+'/calibration_%04d'%index
        try:
            os.mkdir(folder)
            break
        except OSError:
            try:
                os.stat(folder)
            except OSError:
                raise
    else:
        raise OSError('No free calibration directory')
    # 在绘制任何预览叠加层之前保存。
    img.save(folder+'/original.jpg',quality=95)
    crop=img.copy(roi=roi)
    crop.save(folder+'/grid_crop.jpg',quality=95)
    crop=None
    with open(folder+'/roi.json','w') as handle:
        handle.write(ujson.dumps({'roi':roi,'image_size':[WIDTH,HEIGHT],'rotate_180':ROTATE_IMAGE_180,'class':'square','note':'YOLO HBB estimate; verify grid boundaries and grid divisions before precision measurement.'}))
    print('[标定] 标定已保存:',folder)

def main(max_frames=None, capture_background=None):
    global CAPTURE_BACKGROUND, BG_CAPTURING
    if capture_background is not None:
        # 兼容板上 capture_background.py 的调用方式：main(max_frames=90, capture_background=True)
        CAPTURE_BACKGROUND = bool(capture_background)
    sensor=None
    kpu=None
    ai2d=None
    builder=None
    output_tensor=None
    display_ready=False
    media_ready=False
    # UART/控制器在最外层 try 之前初始化，保证任何异常路径下
    # finally 中的 uart.deinit()/controller 引用都不会 NameError
    uart=None
    controller=None
    parser=None
    try:
        try:
            os.stat(MODEL_PATH)
        except OSError:
            raise OSError('Upload best.kmodel to '+MODEL_PATH)
        print('[1] 配置摄像头 RGB565 预览 + RGB 平面推理（yolo 通道 %dx%d）' % (YOLO_CHN_W, YOLO_CHN_H))
        sensor=Sensor(width=WIDTH,height=HEIGHT,fps=SENSOR_FPS)
        sensor.reset()
        sensor.set_framesize(width=WIDTH,height=HEIGHT)
        sensor.set_pixformat(Sensor.RGB565)
        sensor.set_framesize(width=YOLO_CHN_W,height=YOLO_CHN_H,chn=2)
        sensor.set_pixformat(Sensor.RGBP888,chn=2)
        sensor.set_hmirror(ROTATE_IMAGE_180)
        sensor.set_vflip(ROTATE_IMAGE_180)
        # 可选：锁定亮度/白平衡，避免自动曝光在轨迹亮度变化时漂移。
        # 默认关闭以保持原行为；真机调好亮度后在 config.json 开启。
        if LOCK_WHITE_BALANCE:
            try:
                sensor.set_auto_whitebal(False)
                print('[白平衡] 已锁定当前增益')
            except Exception as error:
                print('[白平衡] 锁定失败，保持自动:', error)
        if MANUAL_EXPOSURE_US > 0:
            try:
                sensor.auto_exposure(False)
                print('[曝光] 自动曝光已关闭')
            except Exception as error:
                print('[曝光] 关闭自动曝光失败:', error)
        print('[2] 初始化显示（preview_to_ide=%s）' % str(PREVIEW_TO_IDE))
        if DISPLAY_TARGET=='lcd':
            Display.init(Display.ST7701,to_ide=PREVIEW_TO_IDE)
        elif DISPLAY_TARGET=='virt':
            Display.init(Display.VIRT,width=WIDTH,height=HEIGHT,fps=30,to_ide=PREVIEW_TO_IDE)
        else:
            raise ValueError('DISPLAY_TARGET must be lcd or virt')
        display_ready=True
        MediaManager.init()
        media_ready=True
        print('[3] 启动摄像头')
        sensor.run()
        if MANUAL_EXPOSURE_US > 0:
            try:
                span=sensor.get_exposure_time_range()
                if span:
                    low_us,high_us=sorted((float(span[0]),float(span[1])))
                else:
                    low_us,high_us=1000.0,33000.0
                value=int(clamp(MANUAL_EXPOSURE_US,low_us,high_us))
                sensor.exposure(value)
                print('[曝光] 手动曝光 %d us（范围 %.0f..%.0f）' % (
                    value,low_us,high_us))
            except Exception as error:
                print('[曝光] 设置手动曝光失败，保持自动:', error)
        preview=sensor.snapshot(timeout=1000)
        Display.show_image(preview,x=80 if DISPLAY_TARGET=='lcd' else 0,y=0)
        preview=None
        print('首帧已显示')
        print('[4] 加载 K230 模型:',MODEL_PATH)
        kpu=nn.kpu()
        kpu.load_kmodel(MODEL_PATH)
        print('[5] 配置居中 320x320 RGB 预处理')
        ai2d=nn.ai2d()
        ai2d.set_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8,np.uint8)
        ai2d.set_pad_param(True,[0,0,0,0,40,40,0,0],0,[114,114,114])
        ai2d.set_resize_param(True,nn.interp_method.tf_bilinear,nn.interp_mode.half_pixel)
        builder=ai2d.build([1,3,YOLO_CHN_H,YOLO_CHN_W],[1,3,INPUT_SIZE,INPUT_SIZE])
        output_tensor=nn.from_numpy(np.zeros((1,3,INPUT_SIZE,INPUT_SIZE),dtype=np.uint8))
        print('[6] 检测中：请保持摄像头固定以确认 ROI')
        uart=None
        controller=None
        parser=None
        try:
            uart=init_uart2()
            controller=Requirement5Controller()
            parser=UartFrameParser()
            print('[串口] UART2 TX=Pin%d RX=Pin%d %d 8N1 就绪' % (UART2_TX_PIN, UART2_RX_PIN, UART2_BAUDRATE))
        except Exception as error:
            print('[串口] UART2 初始化失败，仅视觉模式:',error)
        if BG_ENABLED:
            print('[背景] 加载状态:', load_background())
        candidate=None
        confirmed=0
        locked=None
        last_freq=None
        frame_id=0
        votes=[]
        lost=0
        no_grid_frames=0          # D3: YOLO 持续无检测帧计数（回退 GRID_RECT 用）
        last_ctrl_state=None
        bg_captured_run=False
        roi=None
        score=0.0
        last_measure_log=0
        last_sequence=None
        background_mismatch_warned=False
        bg_label=BG_STATUS
        profile_start=ticks_ms()
        profile_frames=0
        infer_total=0
        infer_count=0
        measure_total=0
        measure_count=0
        extract_total=0
        analysis_total=0
        capture_total=0
        display_total=0
        while True:
            os.exitpoint()
            measurement_current=False
            service_uart(uart,parser,controller)
            if controller is not None and controller.sequence!=last_sequence:
                last_sequence=controller.sequence
                votes=[]
                last_freq=None
            # 锁定后保留标定好的 ROI，仅周期性复查摄像头对准。
            # 不在每个预览帧上都跑 YOLO。
            # FPS-OPT: 锁定前隔 GRID_REINFER_EVERY_FRAMES 帧推理（确认网格
            # 不需要每帧）；锁定后每 GRID_RECHECK_FRAMES 帧复查一次。
            grid_checked=((locked is None and frame_id%GRID_REINFER_EVERY_FRAMES==0)
                          or (locked is not None and frame_id%GRID_RECHECK_FRAMES==0))
            if grid_checked:
                stage_start=ticks_ms()
                raw=sensor.snapshot(chn=2,timeout=1000)
                input_tensor=nn.from_numpy(raw.to_numpy_ref())
                builder.run(input_tensor,output_tensor)
                kpu.set_input_tensor(0,output_tensor)
                kpu.run()
                result_tensor=kpu.get_output_tensor(0)
                output=result_tensor.to_numpy()
                roi,score=decode_grid(output)
                # D3 修复(2026-10-22)：YOLO 持续失锁(>=15 帧)且尚未锁定时，
                # 回退真机标定框 GRID_RECT 继续测量，避免网格遮挡/反光/轻微
                # 移动导致系统停摆（原实现 roi=None -> locked 永远 None，
                # GRID_RECT 是死代码）。YOLO 恢复检测后自动切回。
                if roi is None and locked is None:
                    no_grid_frames += 1
                    if no_grid_frames >= 15:
                        roi = GRID_RECT
                        score = 0.0
                        print('[网格] YOLO 失锁 %d 帧，回退 GRID_RECT 标定框继续' % no_grid_frames)
                else:
                    no_grid_frames = 0
                output=None
                result_tensor=None
                input_tensor=None
                raw=None
                infer_total+=ticks_diff(ticks_ms(),stage_start)
                infer_count+=1
                service_uart(uart,parser,controller)
            stage_start=ticks_ms()
            img=sensor.snapshot(timeout=1000)
            capture_total+=ticks_diff(ticks_ms(),stage_start)
            if locked is None:
                confirmed=confirmed+1 if close_roi(candidate,roi) else (1 if roi is not None else 0)
                candidate=roi
                if confirmed>=CONFIRM_FRAMES:
                    locked=roi
                    print('[网格] 锁定成功：GRID_RECT =',locked)
                    if SAVE_LOCKED_CROP:
                        try:
                            save_calibration(img,locked)
                        except Exception as error:
                            print('[网格] 截图保存失败（检测继续）:',error)
            if locked is not None and grid_checked:
                # 2026-10-22：roi=None 表示本轮 YOLO 没检出网格，不等于
                # "网格移动/丢失"。旧实现把 None 当成不匹配 -> lost 每
                # GRID_RECHECK_FRAMES 帧 +1 -> 5 次后丢锁 -> 15 帧后回退
                # GRID_RECT -> 再丢锁，形成约 50% 占空比的 ROI 振荡。
                # 检不到网格时应保持已锁定 ROI（测量本身仍在用 locked）。
                if roi is not None:
                    lost = 0 if close_roi(locked,roi) else lost+1
                    # 只有本轮真的检出并比对过 ROI，才允许据此打断控制；
                    # roi=None 时保持现状，避免反复 vision_lost 打断 HOLD。
                    if lost and controller is not None:
                        controller.vision_lost()
                        last_freq=None
                        votes=[]
                    if lost>=5:
                        locked=None
                        confirmed=0
                        last_freq=None
                        votes=[]
                        print('[网格] 网格丢失，重新获取')
            # 背景差分：请求采集空背景时，网格锁定后开始采集（空屏，无波形）
            if CAPTURE_BACKGROUND and not bg_captured_run and not BG_CAPTURING and locked is not None:
                begin_bg_capture(locked)
                bg_captured_run=True
            if BG_CAPTURING:
                try:
                    accumulate_bg(img)
                except Exception as error:
                    print('[背景] 背景累积异常:', error)
                    BG_CAPTURING=False
            # 差分固定锁定到采集背景时的 ROI（BG_ROI），不跟 YOLO 逐帧锁定的 ROI 走，
            # 避免锁定 ROI 抖动导致背景采样网格错位、静止黄色光标被误检为轨迹。
            # 背景就绪(BG_ENABLED 且已加载 BACKGROUND/BG_ROI)时用 BG_ROI，否则退回当前锁定 ROI。
            analyze_roi = BG_ROI if (BG_ENABLED and BACKGROUND is not None and BG_ROI is not None) else locked
            bg_aligned=not (BG_ENABLED and BACKGROUND is not None and BG_ROI is not None) or close_roi(BG_ROI,locked)
            if locked is not None and not bg_aligned:
                if not background_mismatch_warned:
                    print('[背景] ROI 不匹配：移动摄像头后请重新采集空白背景')
                    background_mismatch_warned=True
                if controller is not None:
                    controller.vision_lost()
                last_freq=None
                votes=[]
            else:
                background_mismatch_warned=False
            active=controller is not None and controller.state not in (controller.WAIT_START,controller.FAILED)
            ready=not active or controller.measurement_ready(ticks_ms())
            due=active or last_freq is None or frame_id%FRAME_ANALYZE_INTERVAL==0
            if locked is not None and not lost and bg_aligned and not BG_CAPTURING and ready and due:
                sample_sequence=controller.sequence if controller is not None else None
                sample_state=controller.state if controller is not None else None
                mode='counts'
                target_shape=None
                if active:
                    if sample_state==controller.MEASURE_X_AMPLITUDE:
                        mode='span'
                    elif sample_state==controller.RAMP_ESTIMATE:
                        # v3：斜坡展开帧——行质心+DFT+LSQ 估频
                        mode='ramp'
                    elif sample_state in (controller.ROTATION_LOCK,controller.LOCAL_SWEEP,controller.PHASE_FINE,controller.AMPLITUDE_FINE,controller.HOLD,controller.DONE):
                        # 旋转锁频/网格扫描/相位/幅度/稳定：需要 div 空间 PCA
                        # 主轴角、fill_ratio 与形状评分（full 测量输出）。
                        # v3.1：LOCAL_SWEEP 用 fill_ratio 判静态细轨迹/旋转
                        # 实心包络，shape 模式每帧都输出 fill_ratio。
                        mode='shape'
                        target_shape=controller.target_shape
                stage_start=ticks_ms()
                last_freq=analyze_frame_for_stage(img,analyze_roi,mode,target_shape)
                measurement_current=True
                measure_total+=ticks_diff(ticks_ms(),stage_start)
                extract_total+=last_freq.get('extract_ms',0)
                analysis_total+=last_freq.get('analysis_ms',0)
                measure_count+=1
                pair=(last_freq['ratio_x'],last_freq['ratio_y']) if last_freq['valid'] and last_freq['ratio_x']>0 and last_freq['ratio_y']>0 else None
                votes.append(pair)
                if len(votes)>3: votes.pop(0)
                last_freq['confirmed']=pair is not None and len(votes)==3 and all(p==pair for p in votes)
                log_due=ticks_diff(ticks_ms(),last_measure_log)>=MEASUREMENT_LOG_MS
                if log_due:
                    last_measure_log=ticks_ms()
                if log_due and last_freq['valid'] and last_freq['span_x_div']>0.0:
                    print('[测量] 有效=%s 已确认=%s 频率比=%d:%d 四边=%s 幅度=%.1fx%.1fdiv fill=%.3f 点数=%d 原因=%s' % (str(last_freq['valid']),str(last_freq['confirmed']),last_freq['ratio_x'],last_freq['ratio_y'],str([last_freq[s] for s in ('left','right','top','bottom')]),last_freq['span_x_div'],last_freq['span_y_div'],last_freq['fill_ratio'],last_freq['trace_pixels'],last_freq['invalid_reason']))
                elif log_due:
                    print('[测量] 模式=%s 有效=%s 四边=%s 幅度=%.2fx%.2fdiv 点数=%d 原因=%s' % (mode,str(last_freq['valid']),str([last_freq[s] for s in ('left','right','top','bottom')]),last_freq['span_x_div'],last_freq['span_y_div'],last_freq['trace_pixels'],last_freq['invalid_reason']))
                service_uart(uart,parser,controller)
                if uart is not None and active and controller.sequence==sample_sequence and controller.state==sample_state and not controller.waiting_for_applied:
                    act=controller.on_measurement(last_freq,time.ticks_ms())
                    if act is not None:
                        send_action(uart,act)
                    # 状态机推进日志：state 变化时打印一次，方便观察
                    # RAMP_ESTIMATE→ROTATION_LOCK→PHASE_FINE→AMPLITUDE_FINE→HOLD
                    if controller.state!=last_ctrl_state:
                        last_ctrl_state=controller.state
                        print("[控制] 状态=%s 锁定=%.3fkHz 稳定=%dms" % (
                            controller.state,
                            controller.locked_frequency_millihz/1.0e6,
                            controller.hold_elapsed_ms(time.ticks_ms())))
            if controller is not None and controller.state!=last_ctrl_state:
                last_ctrl_state=controller.state
                print('[控制] 状态 ->',last_ctrl_state)
            # FPS-OPT: 显示降频。测量/控制仍是每帧执行，只有叠加绘制与
            # LCD show_image 按 DISPLAY_EVERY_N_FRAMES 间隔执行，CPU 拷贝
            # 与 2D 绘制开销减半；显示 15fps 对比赛现场足够流畅。
            display_due = frame_id % DISPLAY_EVERY_N_FRAMES == 0
            if display_due:
                stage_start=ticks_ms()
                img.draw_rectangle(0,0,WIDTH,28,color=(0,0,0),fill=True)
                if roi is not None:
                    img.draw_rectangle(roi,color=(255,255,0),thickness=2)
                if locked is not None:
                    img.draw_rectangle(locked,color=(0,255,0),thickness=2)
                bg_label=BG_STATUS if bg_aligned else 'ROI_MISMATCH_RECAPTURE'
                text=('GRID LOCKED BG:'+bg_label) if locked is not None else 'GRID %.2f  CONFIRM %d/%d'%(score,confirmed,CONFIRM_FRAMES)
                if controller is not None:
                    st=controller.state
                    if st!='WAIT_START':
                        col=(0,255,0) if st in ('HOLD','DONE') else (255,255,0)
                        status_text='CTRL '+st
                        if st==controller.FAILED:
                            status_text+=' E%d'%controller.failure_code
                        img.draw_string_advanced(4,74,18,status_text,color=col)
                img.draw_string_advanced(4,2,18,text,color=(0,255,0))
                if locked is not None and last_freq is not None:
                    m=last_freq
                    ok=m['confirmed']
                    label='fx:fy=%d:%d %s'%(m['ratio_x'],m['ratio_y'],'CONFIRMED' if ok else 'UNCONFIRMED')
                    if ok and KNOWN_Y_HZ is not None:
                        label+=' fx=%.2fHz'%(float(KNOWN_Y_HZ)*m['ratio_x']/m['ratio_y'])
                    img.draw_string_advanced(4,30,18,label,color=(0,255,0) if ok else (255,100,0))
                    img.draw_string_advanced(4,52,16,'L/R/T/B %d/%d/%d/%d %s'%(m['left'],m['right'],m['top'],m['bottom'],str(m['invalid_reason'])),color=(255,255,255))
                    if measurement_current and m['valid']:
                        for x,y in m.get('contacts',[]):
                            img.draw_circle(x,y,3,color=(255,0,255),thickness=1)
                    bounds=m.get('pixel_bounds') if measurement_current else None
                    if bounds:
                        x1,x2,y1,y2=bounds
                        img.draw_rectangle(int(x1+analyze_roi[0]),int(y1+analyze_roi[1]),int(x2-x1+1),int(y2-y1+1),color=(0,255,255),thickness=1)
                Display.show_image(img,x=80 if DISPLAY_TARGET=='lcd' else 0,y=0)
                display_total+=ticks_diff(ticks_ms(),stage_start)
            img=None
            frame_id+=1
            profile_frames+=1
            service_uart(uart,parser,controller)
            if max_frames is not None and frame_id>=max_frames:
                print("[冒烟] 测试完成 帧数=",frame_id)
                break
            now=ticks_ms()
            elapsed=ticks_diff(now,profile_start)
            if elapsed>=2000:
                print('性能 fps=%.1f 推理=%.1fms 测量=%.1fms 提取=%.1fms 分析=%.1fms 测量频率=%.1fHz 采集=%.1fms 显示=%.1fms 掩码=%s 背景=%s 状态=%s' % (
                    1000.0*profile_frames/max(1,elapsed),infer_total/max(1,infer_count),
                    measure_total/max(1,measure_count),extract_total/max(1,measure_count),
                    analysis_total/max(1,measure_count),1000.0*measure_count/max(1,elapsed),
                    capture_total/max(1,profile_frames),
                    display_total/max(1,profile_frames),MASK_ACCESS_MODE,bg_label,
                    controller.state if controller is not None else 'VISION_ONLY'))
                profile_start=now
                profile_frames=0
                infer_total=0
                infer_count=0
                measure_total=0
                measure_count=0
                extract_total=0
                analysis_total=0
                capture_total=0
                display_total=0
                gc.collect()
    except KeyboardInterrupt:
        print('已停止（IDE 中断）')
    except BaseException as error:
        sys.print_exception(error)
    finally:
        if uart is not None:
            try:uart.write('$STOP,0\r\n')
            except Exception:pass
            try:uart.deinit()
            except Exception:pass
        if sensor is not None:
            try:sensor.stop()
            except Exception:pass
        if display_ready:
            try:Display.deinit()
            except Exception:pass
        builder=None
        ai2d=None
        output_tensor=None
        kpu=None
        gc.collect()
        try:nn.shrink_memory_pool()
        except Exception:pass
        if media_ready:
            try:
                os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
                time.sleep_ms(100)
                MediaManager.deinit()
            except Exception:pass

if __name__=='__main__':
    main()
