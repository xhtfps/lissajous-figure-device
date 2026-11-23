# 固定周期短斜坡测频方案 可行性分析（2026-10-22）

> 对象：要求 5（断开信号源、纯摄像头闭环、控制时间≤10s、稳定≥5s）
> 方法：装置 Y 通道输出已知周期的锯齿斜坡，把示波器 XY 显示中的 X 输入正弦"沿 Y 轴展开"成时间波形，相机从图像计数/拟合周期数 N，得 fx = N / T_ramp。
> 现状：K230 主程序 v2 全盲 1kHz 粗扫存在"候选遗漏 + 6.7s 预算"硬伤；本方案为替代粗测频手段，与既有 ROTATION_LOCK / PHASE_FINE / AMPLITUDE_FINE / HOLD 状态机衔接。

---

## 一、结论（先读）

**可行，且是当前能找到的最优替代路线；但"可行"的边界必须划清楚：**

1. **数学上闭环**。斜坡展开 + 周期估计（DFT 粗估 + 最小二乘正弦拟合）能把 fx 估到亚周期精度；CdyWork 同源实现的 991 点（1k~100kHz、100Hz 步进）仿真回归 **100% 命中 100Hz 档**（含网格线、模糊、噪声、透视、多帧累积、±5ppm 源时钟误差）。见 §四.1。
2. **不是"唯一解"而是"粗锚点"**。斜坡单次估计的相对精度受"每屏可数周期数 N"限制（约 ±1/(2N)）；100kHz 时 N≤50 → 绝对误差 ±1kHz 量级。**必须靠两步收敛**：斜坡粗估 → 按 100Hz 网格设定输出频率 → 既有旋转锁频（翻转周期）精调 Δf→0。这是 CdyWork 仿真回归与 IIETA 翻转周期法各自解决的问题，合并后闭环。
3. **关键几何洞察（CdyWork 提出，必须采用）**：题面所有允许频率都是 100Hz 整数倍，取**探测周期 = 10ms（=100Hz 周期的整数倍）**，每次斜坡起点处输入正弦相位相同 → 多帧/余晖叠加轨迹稳定不糊，计数才有意义。这是"短斜坡"能成像的前提。
4. **硬件前提尚未满足，是本方案最大风险**：Y 通道当前由 AD9959（经 I/V+OPA1612）驱动；斜坡需要 **F407 内置 DAC PA4 输出接入示波器 Y**。DAC.c 已有 PA4 锯齿波 DMA 接口（表长 32~128、最高 4MHz 采样率，可覆盖 100Hz~100kHz 斜坡），但**是否接到 Y 通道未确认**。未接线则 RAMP 阶段无轨迹，必须回退原全盲搜。
5. **仿真≠真机**：CdyWork 作者明确声明仿真未覆盖示波器余辉算法、相机摩尔纹/眩光、DAC 建立时间；本方案在上板前必须完成 §五 的台架实验清单。

---

## 二、方法原理与数学边界

### 2.1 斜坡展开（把 XY 显示变成"自制时基"）

示波器 XY 模式下，X=输入正弦，Y=装置输出的线性斜坡 u_y(t)=k·t（周期 T_ramp，斜率 k=2·Vpp·f_ramp 已知）。屏幕上轨迹：

```
x(u) = A·sin(2π·fx·t(u)),  t(u) = u·T_ramp,  u∈[0,1] 为归一化纵向位置
→ x(u) = A·sin(2π·N·u),   N = fx·T_ramp = fx / f_ramp
```

即：**一个斜坡窗口内可见的正弦周期数 N 在数值上等于 fx·T_ramp**（T_ramp=1ms 时 N = fx[kHz]）。测频问题变成"从一条垂直展开的正弦曲线估计其空间频率 N"。这是示波器教材"锯齿波线性扫描电压 ux=kt 作时基"原理在 XY 模式的直接应用（武汉大学物理实验讲义原文）。

### 2.2 精度边界（为什么单次计数不够、为什么要拟合）

- 直接数波峰/过零：±0.5 周期 → Δfx = ±0.5/T_ramp。1ms 斜坡 → ±500Hz；0.5ms → ±1kHz。**不够**。
- DFT（周期图）粗估 + 最小二乘正弦拟合（含 sin/cos/直流/线性漂移四项）把 N 估到**亚周期**（CdyWork 用 65536 点 rfft 定粗峰 + 黄金分割细化 LSQ 拟合）。在 380px 有效纵向区、N≤50（0.5ms 斜坡）时，拟合误差远小于 0.5 周期 → 100Hz 档可稳定分辨（其 991 点回归 100% 命中即为证明）。
- **用"拟合"而非"数数"是本方案的核心**：全部采样点参与估计，等效分辨率大幅超过像素/周期比。

### 2.3 自适应档位（CdyWork 的 70kHz 分界，直接采用）

| 斜坡时长 T_ramp | N=fx·T_ramp 覆盖 | 适用频段 | 备注 |
|---|---|---|---|
| 1ms | 1~100 | 全频段第一拍 | fx≤70kHz 一锤定音 |
| 0.5ms | 0.5~50 | ≥70kHz 第二拍 | 像素/周期翻倍 |

> CdyWork 原文：探测波形 10ms 重复、1ms 可见上升斜坡、其余 9ms 停在屏外；>70kHz 用第二段 0.5ms 斜坡。**10ms 重复周期 = 100Hz 网格整数倍，保证每次斜坡起点输入相位一致**，多帧叠加不糊。

### 2.4 两步收敛闭环（本方案与既有状态机的衔接）

```
MEASURE_X（测X幅度，5帧）
  → RAMP_ESTIMATE（新）：$RAMP 斜坡 → 1~2 帧 → DFT+LSQ 估 N → fx_est=N/T_ramp
     （N 越界则换档重拍，最多 3 次：1ms → 0.5ms → 0.25ms）
  → 输出 fx_est 四舍五入到 100Hz 网格（AD9959 毫赫兹命令，残差≤50Hz）
  → 目标形状 2 帧确认（复用 SWEEP_CONFIRM 判据）
     ├ 命中 → ROTATION_LOCK（残差≤50Hz，主轴旋转速率测 Δf → 跳频，≤4轮压到<1Hz）
     ├ 未命中 → 局部 ±10 档 100Hz 细扫（≤21 点 ≈1.5s，必中网格点）
     └ RAMP 失败（无轨迹/F407 报错/拟合超界）→ 回退 SWEEP_GLOBAL 全盲搜
  → PHASE_FINE → AMPLITUDE_FINE → HOLD(5s) → DONE
```

**10s 预算（理论最坏）**：MEASURE_X ~0.4s + RAMP 两拍 ~0.6s + 网格确认 ~0.3s + 局部细扫兜底 ~1.5s + ROTATION_LOCK ~0.7s + PHASE ~0.9s + AMPLITUDE ~0.3s ≈ **4.7s < 10s**。相比全盲 1kHz 粗扫（6.7s 采样 + 候选确认，且会整体漏候选），本方案从"概率命中"变成"有锚点的确定性收敛"。

---

## 三、证据清单（论坛 + GitHub + 学术 + 教材）

| # | 来源 | 类型 | 与本方案的关系 | 验证等级 |
|---|---|---|---|---|
| 1 | **CdyWork/2026F-lissajous-controller**（89★，Tang Primer 25K + STM32F407 + Pi） | GitHub 开源 | V2 增加视觉探测模式 PROBE：10ms 周期探测锯齿波、100/200/500/1000/2000/5000µs 六档斜坡覆盖 1k~100kHz 视觉粗测频；`q5_sawtooth_vision_sim` 为**本方案的同源实现**（FFT 粗估 + LSQ 正弦拟合 + 991 点 100% 档命中回归） | **已查证**（读仓库 README、PROJECT_2026F.md、vision_sim.py 全文） |
| 2 | **IIETA 2023《Enhanced Frequency Measurement via Lissajous Figure Flipping Periods》**（Traitement du Signal 40(3):961-970） | 学术论文 | 翻转周期法：相机逐帧归一化面积 NA(i) 负峰 = 翻转时刻 → Δf=1/(m·T)。本方案 ROTATION_LOCK 精调的学术依据；对 1:1 和 2:1 均成立 | **已查证**（前轮已读全文，本轮复核摘要） |
| 3 | **中国专利 CN105004921《基于李沙育图形的异频高精度频率测量方法》** | 专利 | NA(i) 重合度最大帧标记翻转周期——翻转周期法在中国的同族专利，印证该路线可专利级实现 | 已查证（专利正文摘录） |
| 4 | **EEVblog 1751《Oscilloscope Phase Measurement Masterclass》**（论坛帖 7429 次阅读；17:55 起 XY 模式李萨如测量；R&S 产品经理回帖并附李萨如系列视频） | 论坛 | 佐证 XY 模式李萨如测量是行业标准实操；**但内容是相位测量，不是斜坡测频**——正如用户判断：只适合做"设备差异提醒"，不能作为算法验证 | **已查证**（论坛帖全文） |
| 5 | **TestAndMeasurementTips《Lissajous patterns: using a scope for display signals》** | 行业媒体 | XY 模式频率比/相位测量的原理综述（含 a/b 频率比对形状的控制） | **已查证**（全文） |
| 6 | 武汉大学《实验3-9 示波器的原理及应用》 | 物理实验教材 | "水平偏转（X轴）加入锯齿波线性扫描电压 ux=kt"——**锯齿波=时间轴**是示波器时基的本源原理，斜坡展开的直接理论依据 | 已查证（教材原文） |
| 7 | 哈尔滨工业大学《数字示波器的原理与应用》 | 物理实验教材 | 李萨如图切点法测频（fx:fy = ny:nx）——比例测频的传统方法，与翻转周期法互补 | 已查证 |
| 8 | E谷电赛《李萨如图视觉频率检测方案》（BV1ekGF6oEec，33:40，全开源带上位机） | B站开源课程 | 断信号源后纯视觉反推频率的主流公开路线；未公开具体算法，方向与本方案一致 | 已查证（标题/标签） |
| 9 | 2026 获奖队伍公开动态（山东省一视频、满分演示、"第五问实在难度不小"） | B站 | 要求 5 是全队级难点；无公开代码 | 已查证 |

### 3.1 反面证据（必须写入判断）

- **没有**任何公开仓库完成了"斜坡测频 + 实机示波器/相机闭环"的端到端验证；CdyWork 明确声明仿真未覆盖示波器余辉算法、相机摩尔纹/眩光、DAC 模拟建立时间。
- EEVblog 等论坛讨论集中在 XY 模式相位/频率比的**手动**测量，未见"自动斜坡展开测频"的实机复现帖。
- 结论：**方案有同源实现和数学保证，但"本装置 + 本示波器 + 本相机"的真机有效性必须自行台架验证**，不能把 CdyWork 的仿真回归当作实机通过证明。

---

## 四、与本装置现状的差距清单（改动点）

| 项 | 现状 | 本方案需要 | 改动 |
|---|---|---|---|
| Y 通道驱动源 | AD9959（I/V+OPA1612）→ 示波器 Y | 斜坡期需 F407 DAC PA4 → Y | **硬件**：PA4 接 Y（或模拟开关/继电器切换）；未接线则回退全盲搜 |
| F407 协议 | $START/$SET/$APPLIED/$LOCKED/$DONE/$FAILED | 新增 $RAMP,seq,probe,saw_hz,amp_mv | vision_slave 解析 + f_app 处理（停 AD9959→开 DAC 锯齿；下条 SET 反向） |
| K230 状态机 | MEASURE_X→SWEEP_LOCAL→SWEEP_GLOBAL→ROTATION_LOCK... | 新增 RAMP_ESTIMATE + 网格确认 + 局部细扫兜底 | main.py 新状态 + 斜坡图像分析函数 |
| 轨迹提取 | collect_yellow_points 整屏采样 | 斜坡模式按行取质心（去回扫竖线、多帧叠加按行平均） | 新函数 analyze_ramp_frame |
| 周期估计 | 无 | DFT 粗估 + LSQ 拟合（纯 Python/MicroPython 可跑） | 新函数 estimate_ramp_cycles |
| 配置 | config.json 若干键 | RAMP_ENABLED、RAMP_SAW_HZ、RAMP_AMP_MV 等 | config.json + 常量默认值 |
| 测试 | test_v2_state_machine_smoke.py 绿 | 增斜坡合成数据单测 + 状态机 RAMP 路径 | tests/ |

## 五、台架实验清单（上板前必须做，按顺序）

1. **PA4→Y 接线确认**：示波器 Y 通道改接 F407 PA4（或切换开关）；菜单6 现有 `F_APP_DAC_SAWTOOTH_TEST` 宏改 1 发 1kHz 锯齿，确认屏上出现斜坡扫过的正弦展开轨迹、幅度≈6.6div（DAC 满幅 3.3V@0.5V/div）。
2. **10ms 相位对齐验证**：设信号源 12.3kHz，斜坡 1ms/10ms 重复；确认多帧叠加轨迹**不糊**（若糊：检查斜坡重复周期是否恰为 10ms、示波器余晖/持久模式设置）。
3. **曝光与斜坡时间窗匹配**：实测相机曝光；确认 1ms（及 0.5ms）斜坡在单帧内可见完整轨迹（不行则依赖示波器持久模式，需重测计数稳定性）。
4. **DFT+LSQ 精度标定**：信号源扫 1/5/10/50/70/100kHz 各 3 个 100Hz 档，记录 fx_est 与真值误差；目标 <±50Hz（不达标先查 GRID_RECT/行质心提取/去回扫）。
5. **ROTATION_LOCK 衔接**：fx_est 置网格后残差≤50Hz 时，确认主轴旋转锁频 ≤4 轮收敛（发 +1Hz 探针验证 ROTATION_SIGN 与 ROTATION_HZ_PER_TURN_DEG）。
6. **10s 全程计时**：从 $START 到 $LOCKED 记录各阶段耗时；确认 <10s，留 2s 余量给兜底路径。
7. **失败路径演练**：断开 PA4（模拟未接线），确认 K230 自动回退 SWEEP_GLOBAL 且不超时崩溃。

## 六、来源 URL

- https://github.com/CdyWork/2026F-lissajous-controller （README / PROJECT_2026F.md / q5_sawtooth_vision_sim/vision_sim.py）
- https://iieta.org/journals/ts/paper/10.18280/ts.400312 （翻转周期法论文）
- https://www.xjishu.com/zhuanli/52/cn105004921.html （CN105004921 专利）
- https://www.eevblog.com/forum/blog/eevblog-1751-oscilloscope-phase-measurement-masterclass/ （EEVblog 1751 论坛帖）
- https://www.testandmeasurementtips.com/using-scope-display-lissajous-patterns/
- https://wlsyzx.whu.edu.cn/__local/C/03/79/EC9C30DB6DFEA8F45162A8CD63E_52463843_447A85.pdf （武大示波器讲义）
- https://elcap.hit.edu.cn/_upload/article/files/72/be/086e12d94921a6875516f06450f3/ab5bd527-2c09-4b51-a149-73ce6abc41c4.pdf （哈工大示波器讲义）
- https://www.bilibili.com/video/BV1ekGF6oEec/ （E谷视觉频率检测方案）
