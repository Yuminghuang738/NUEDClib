# NUEDClib 系统架构与接口规范

## 系统概览

```
┌─────────────────────────────────────────────────────────────────┐
│                        OpenMV Cam                              │
│  main.py / get_regression.py / template_matching.py / ...       │
│  ┌──────────┐   ┌─────────────────┐   ┌──────────┐   ┌─────────────┐ │
│  │ 传感器    │ → │ 融合检测        │ → │ 偏移计算  │ → │ 帧打包      │ │
│  │ snapshot  │   │ 霍夫圆 + 高光    │   │ X轴 only  │   │ struct.pack │ │
│  └──────────┘   └──────────┘   └──────────┘   └──────┬──────┘ │
│                                                       │ UART TX│
└───────────────────────────────────────────────────────┼────────┘
                                                        │
                                           5-byte frame │ 115200bps
                                                        │
┌───────────────────────────────────────────────────────┼────────┐
│                        MSPM0G3507                      │ RX     │
│  ┌──────────┐   ┌──────────────┐   ┌──────────────────┴──────┐ │
│  │ R 电机    │ ← │  P 控制器     │ ← │  UART 解析              │ │
│  │ 绳长控制  │   │  vision.c    │   │  uart.c                 │ │
│  └──────────┘   └──────────────┘   └─────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

---

## 一、UART 协议（OpenMV ↔ MSPM0 之间的契约）

### 1.1 物理层

| 参数 | 值 |
|------|-----|
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验位 | None |
| 停止位 | 1 |
| 方向 | OpenMV TX(P1) → MSPM0 RX(P0)，单向 |
| 帧间隔 | 无要求（每帧独立，状态机自动同步） |

### 1.2 帧格式（5 字节）

```
 Byte 0   Byte 1    Byte 2     Byte 3     Byte 4
┌────────┬─────────┬──────────┬──────────┬────────┐
│  0xAA  │ status  │ offset_x │ offset_y │  0xBB  │
│ 帧头   │ 状态字节 │ X偏移    │ Y偏移    │ 帧尾   │
│ uint8  │ uint8   │ int8     │ int8     │ uint8  │
└────────┴─────────┴──────────┴──────────┴────────┘
```

### 1.3 status 字节位定义

```
 Bit  7   6   5   4   3     2     1     0
┌────┬────┬────┬────┬────┬─────┬─────┬─────┐
│  0 │  0 │  0 │  0 │  0 │ conf│ conf│valid│
│  保留                                  │
└────┴────┴────┴────┴────┴─────┴─────┴─────┘
```

| 位 | 名称 | 说明 |
|----|------|------|
| bit 0 | `data_valid` | 0=未检测到目标, 此帧的 offset 无效; 1=检测到目标, offset 有效 |
| bit 1-2 | `confidence` | 0~3, 检测置信度。3=高度可信, 2=基本确定, 1=勉强检测 |
| bit 3-7 | reserved | 保留, 填 0 |

### 1.4 偏移值范围

QQVGA = 160×120 像素，画面中心 = (80, 60)。平衡滚球任务仅使用 X 轴（沿水管方向），Y 轴恒为 0：

| 字段 | 类型 | 范围 | 含义 |
|------|------|------|------|
| `offset_x` | int8 | -80 ~ +79 | 正值=钢球在参考点右侧, 负值=左侧 → 绳长调节 |
| `offset_y` | int8 | 0 | **恒为 0**（水管横向放置，Y 轴不可控） |

### 1.5 data_valid=0 时的约定

- `offset_x` 和 `offset_y` 必须填 `0`（安全默认值）
- MSPM0 收到 `data_valid=0` 后不更新 `last_dev` 和 `last_confidence`，EMA 向 0 自然衰减
- UART 物理断线时 `last_frame_ms` 停止更新，200ms 后超时停机

---

## 二、OpenMV 端接口规范

### 2.1 任务脚本必须遵守的约定

每个追踪任务的脚本必须：

1. **输出 5 字节帧**：`struct.pack("<BBbbB", 0xAA, status, offset_x, offset_y, 0xBB)`
2. **正确设置 status 字节**：
   - 检测到目标 → `data_valid=1`, `confidence` 按任务自定义规则填入 1~3
   - 未检测到目标 → `data_valid=0`, `confidence=0`, `offset_x=offset_y=0`
3. **每帧都发送**：MSPM0 依赖持续收帧来判断连接正常（200ms 超时）

### 2.2 传感器配置（可任务自定义）

| 参数 | 循迹任务 | 颜色追踪任务 | 说明 |
|------|---------|-------------|------|
| `pixformat` | GRAYSCALE | RGB565 | 像素格式 |
| `framesize` | QQVGA | QQVGA | **固定**，协议偏移范围依赖此分辨率 |
| `auto_gain` | False | False | **必须关闭**，否则阈值失效 |
| `auto_whitebal` | False | False | **必须关闭** |
| `auto_exposure` | False | False | **必须关闭**，曝光时间按场景设定 |
| `exposure_us` | 25000 | 15000 | 按光照调参 |

### 2.3 检测算法（可任务自定义）

OpenMV 负责：
- 选择合适的 `find_blobs` / `find_template` / `get_regression` 等
- 从检测结果中选出**唯一**要追踪的目标（通常取最大 blob / 最佳匹配）
- 做基本的有效性过滤（密度、尺寸、匹配分数等）

OpenMV **不负责**：
- 控制算法（那是 MSPM0 的事）
- 平滑滤波（那是 MSPM0 的事）
- 电机逻辑

### 2.4 confidence 计算指南

confidence 的**语义**是"这个偏移值有多可信"，不是"目标有多大"。各任务自行定义阈值，但应遵循：

| conf | 语义 | 典型条件 |
|------|------|---------|
| 3 | 高度可信 | 目标清晰、无遮挡、特征完整 |
| 2 | 基本确定 | 目标部分可见或有轻微干扰 |
| 1 | 勉强检测 | 目标在视野边缘、或被部分遮挡 |

MSPM0 会根据 confidence 缩放电机速度：conf=3 → 全速, conf=2 → 75%, conf=1 → 50%。

### 2.5 当前已实现的任务

| 任务 | 文件 | 像素格式 | 检测算法 | confidence 来源 |
|------|------|---------|---------|----------------|
| 红色颜色追踪 | `main.py` | RGB565 | `find_blobs` | blob 占画面比 |
| 线性回归循迹 | `get_regression.py` | GRAYSCALE | `get_regression` | 回归强度 magnitude |
| 模板匹配追踪 | `template_matching.py` | GRAYSCALE | `find_template` | 相关系数 score |
| 灰度 blob 检测 | `find_blobs.py` | GRAYSCALE | `find_blobs` | 未改造 |
| **平衡滚球** | `main_balance_ball.py` | RGB565 | **霍夫圆 + 高光融合** | 融合分数 |

---

## 三、MSPM0 端接口规范

### 3.1 文件与职责

```
MSPM0G3507/
├── main.c                          # 入口, 调用 process_deviation()
├── User/
│   ├── uart.h / uart.c             # UART 收帧 + 协议解析
│   ├── trace.h / trace.c           # 红外循迹（独立子系统，当前未使用）
│   └── motor.h / motor.c           # 驱动轮电机（独立子系统）
│
OpenMV/
├── vision.h / vision.c             # 云台 P 控制器（EMA + 迟滞死区 + 置信度增益）
├── gimbal_motor.h / gimbal_motor.c # 云台步进电机驱动
├── main.py                         # 颜色追踪任务
├── get_regression.py               # 线性回归循迹任务
├── template_matching.py            # 模板匹配追踪任务
└── find_blobs.py                   # 灰度 blob 检测（未改造）
```

### 3.2 UART 层 (uart.h / uart.c)

**公共接口：**

```c
// 轮询 UART FIFO 并驱动状态机，由 UART_get_deviations 内部调用
void UART_poll_rx(void);

// 尝试获取一帧数据（非阻塞）
// 返回 true 表示收到完整且校验通过的帧
// out_status: 帧的 status 字节（原始值，调用方自行解析位）
// out_x:      offset_x (int8)
// out_y:      offset_y (int8)
bool UART_get_deviations(uint8_t *out_status, int8_t *out_x, int8_t *out_y);
```

**实现逻辑：**

1. 调用 `UART_poll_rx()` 从 FIFO 读取字节
2. 状态机：`IDLE → STATUS → DATA_X → DATA_Y → TAIL → IDLE`
3. 帧校验：
   - 帧头 `rx_buffer[0] == 0xAA`
   - 帧尾 `rx_buffer[4] == 0xBB`
   - 长度 `rx_index == 5`
4. 校验失败 → 丢弃帧，状态机回到 IDLE 等待下一帧头
5. **不关心** status/x/y 的语义，纯粹传递

**内部常量：**

```c
#define RX_BUF_SIZE 6   // 接收缓冲区大小（5 字节帧 + 1 字节冗余）
```

### 3.3 控制层 (vision.h / vision.c)

**公共接口：**

```c
// 主循环每轮调用一次。内部完成：
//   1. 读取 UART 帧
//   2. 解析 data_valid / confidence
//   3. EMA 低通滤波（有效帧跟踪测量值，无效帧向 0 衰减）
//   4. 迟滞死区判断
//   5. PD 控制器 (位置 + 速度阻尼)
//   6. 步进定位输出 (固定速度, 变角度)
void process_deviation(void);
```

**配置宏（可调参数）：**

```c
// ── PD 控制器 ──
#define VISION_KP_X               2       // 比例增益 (P — 位置纠正)
#define VISION_KD_X               2       // 微分增益 (D — 速度阻尼)

// ── 步进定位 ──
#define VISION_K_ANGLE_X          2.0f    // deg/pixel: 像素→角度映射
#define VISION_STEP_SPEED_X       60      // 固定步进速度 (deg/s)
#define VISION_MAX_ANGLE_PER_FRAME 12     // 单帧最大角度 (防止过冲)

// ── 死区 ──
#define VISION_DEAD_ZONE_ENTER_X  2       // 进入死区阈值 (像素)
#define VISION_DEAD_ZONE_EXIT_X   4       // 退出死区阈值 (像素)

// ── 共用 ──
#define VISION_TIMEOUT_MS         200     // 失联超时 (ms)
#define VISION_EMA_ALPHA_Q8       64      // EMA 系数 Q8: 64/256=0.25
```

**置信度增益表：**

```c
static const uint16_t CONF_GAIN_Q8[] = {0, 128, 192, 256};
//   conf=0:   0/256 = 0.00x (不应出现，data_valid=0 时不进入控制)
//   conf=1: 128/256 = 0.50x (勉强检测 → 半幅度)
//   conf=2: 192/256 = 0.75x (基本确定 → 中幅度)
//   conf=3: 256/256 = 1.00x (高度可信 → 全幅度)
```

**控制流程图：**

```
process_deviation()
  │
  ├─ UART_get_deviations(&status, &x, &y)   // y 忽略
  │    │
  │    ├─ 有帧 → 更新 last_frame_ms
  │    │         │
  │    │         ├─ data_valid=1 → 更新 last_dev_x, last_confidence, EMA 跟踪 X
  │    │         └─ data_valid=0 → EMA 向 0 衰减 (每帧 ×0.75)
  │    │
  │    └─ 无帧 → 不更新（last_frame_ms 不动 → 200ms 后超时）
  │
  ├─ 超时检查 (sys_tick_ms - last_frame_ms > 200ms)
  │    └─ 超时 → R 电机停机 + EMA/PD 复位 + return
  │
  └─ control_axis(filtered_dev, confidence)   // PD + 步进定位
       │
       ├─ 迟滞死区判断 (同前)
       │
       ├─ velocity = deviation - last_error_x    // 速度估计
       ├─ output = KP * deviation + KD * velocity  // PD 输出
       │    │
       │    ├─ 球远离中心, 速度同向 → P+D 叠加, 强力纠正
       │    └─ 球回正中, 速度反向 → D 抵消 P, 减轻力度防过冲
       │
       ├─ target_angle = K_ANGLE * |output|       // 像素 → 角度
       ├─ clamp(target_angle, 0, MAX_ANGLE_PER_FRAME)
       ├─ target_angle *= CONF_GAIN_Q8[confidence] >> 8
       ├─ set_dir(R, deviation > 0 ? FWD : REV)
       ├─ set_speed(R, STEP_SPEED)   // 固定速度
       └─ set_angle(R, target_angle) // 走完自动停 (ISR 计数)
```

### 3.4 EMA 衰减机制

当 `data_valid=0`（本帧未检测到目标）且 EMA 已初始化时：

```c
// 用虚拟测量值 0 驱动衰减，每帧乘以 0.75
ema_x_q8 -= ema_x_q8 >> 2;   // ema = ema * 3/4  (仅 X 轴)
```

**解决的问题：**

| 场景 | 改前 | 改后 |
|------|------|------|
| 单帧噪点误检 | EMA 锁死在非零值，电机永远转 | EMA 4 帧后衰减到 0，电机 130ms 内停机 |
| 目标短暂消失 | EMA 保持旧值，电机继续转 | EMA 缓慢衰减，恢复后瞬间拉回 |
| 正常追踪中丢失目标 | 只有靠 UART 断线超时才停 | EMA 约 10 帧 (~330ms) 自然归零停机 |
| 开机无目标（第一帧误检） | EMA 初始化到误检值 → 锁死 | 第二帧 data_valid=0 → EMA 衰减 → 停机 |

### 3.5 云台电机层 (gimbal_motor.h / gimbal_motor.c)

平衡滚球任务仅使用 R 电机（绳长控制），L 电机保留不使用。

**连续转动模式：**

`vision.c` 启动时调用 `gimbal_motor_set_continuous(R, 1)`，设置静态标志位 `gimbal_continuous_r = 1`。每次 PWM 脉冲触发 ISR 时：

```
ISR → 检查 gimbal_continuous
        ├─ =1 → break（跳过步数计数，不停机）← 绳长控制用这个
        └─ =0 → step_remain-- → 到 0 自动停机 ← 角度模式用这个
```

连续模式下停机完全由应用层控制（`control_axis` 的迟滞死区判断 + `process_deviation` 的超时判断）。

**R 电机方向修正：**

R 电机（垂直轴）的 GPIO 方向逻辑已在 `gimbal_motor_set_dir` 中交换 `setPins` ↔ `clearPins`，以匹配实际物理安装方向。`gimbal_motor.h` 的宏定义未变，上层 `vision.c` 对 FORWARD/REVERSE 的理解保持一致。

### 3.6 红外循迹子系统 (trace.h / trace.c)（独立，当前停用）

```c
void trace_get_value(void);   // 读取 4 路红外传感器 → trace_data[4]
void trace_motor(void);       // 基于质心误差的线跟随控制
```

与视觉子系统**互斥使用**——`main.c` 的 `while(1)` 中只调用其中一个。

---

## 四、调用约定与时序契约

### 4.1 帧率与实时性

| 项目 | 约束 | 说明 |
|------|------|------|
| OpenMV 帧率 | 无硬性要求 | 30fps 左右正常，掉到 10fps 仍可工作 |
| MSPM0 主循环频率 | 越快越好 | `while(1)` 无延时，`process_deviation()` 非阻塞 |
| 超时阈值 | 200ms | 超过此时间无**任何**帧 → 停机 |
| EMA 平滑窗口 | α=0.25 | 约 4 帧收敛到新值 63%，约 12 帧收敛到 95% |
| EMA 衰减速度 | α=0.25 向 0 | 约 4 帧衰减 63%，约 10 帧 (~330ms) 进入死区 |

### 4.2 启动时序

```
T=0     OpenMV 上电 → sensor.reset() → skip_frames(2000ms)
        MSPM0 上电  → SYSCFG_DL_init() → motor_init() → gimbal_motor_init()
T=2s    OpenMV 开始发送帧
        MSPM0 process_deviation() 首次调用 → set_continuous()
T=2s+   正常跟踪
```

MSPM0 在 OpenMV 发帧之前（前 2 秒）处于超时状态，电机会保持停机。

### 4.3 异常恢复

| 异常 | OpenMV 行为 | MSPM0 行为 | 恢复 |
|------|-----------|-----------|------|
| 目标消失 | data_valid=0, offset=0 | EMA 向 0 衰减，~330ms 进入死区停机 | 目标重现 → 立即恢复 |
| 单帧噪点误检 | 下一帧纠正为 data_valid=0 | EMA 短暂跳起 → 4 帧内衰减回 0 | 自动 |
| 短暂遮挡 | data_valid=0 | EMA 缓慢衰减，恢复后瞬间拉回 | 自动 |
| UART 断线 | 无帧发出 | last_frame_ms 不动 → 200ms 超时停机 | 重连后自动恢复 |
| OpenMV 重启 | 2s 后恢复发帧 | 200ms 超时 → 停机 → 恢复后自动跟踪 | 自动 |
| MSPM0 重启 | 不受影响 | 重新初始化 | 自动 |
| 全黑/过曝 | data_valid=0 | EMA 衰减 → 停机 | 光照恢复 → 自动恢复 |
| R 电机方向反 | — | GPIO 电平已在 `gimbal_motor_set_dir` 修正 | — |

---

## 五、新增追踪任务的 checklist

在 OpenMV 端实现新的追踪任务时，确保：

- [ ] 像素格式与阈值匹配（GRAYSCALE 用单通道，RGB565 用 LAB 六通道）
- [ ] `framesize` 保持 QQVGA（偏移范围硬依赖 160×120）
- [ ] auto_gain / auto_whitebal / auto_exposure 全部 False
- [ ] 检测到目标时：`data_valid=1`, `confidence` 填入 1/2/3
- [ ] 未检测到目标时：`data_valid=0`, `confidence=0`, `offset_x=offset_y=0`
- [ ] 帧打包格式：`struct.pack("<BBbbB", 0xAA, status, offset_x, offset_y, 0xBB)`
- [ ] 每帧都发送（包括无效帧），保持 MSPM0 侧连接存活
- [ ] MSPM0 端 **不需要任何修改**

---

## 六、版本历史

| 日期 | 改动 | 涉及文件 |
|------|------|---------|
| 2026-07-19 | 初始版本：5 字节协议 + EMA + 迟滞死区 | main.py, uart.c, vision.c |
| 2026-07-19 | 颜色追踪迁移、confidence 增益缩放 | main.py, vision.c |
| 2026-07-19 | get_regression.py / template_matching.py 适配协议 | get_regression.py, template_matching.py |
| 2026-07-19 | EMA data_valid=0 衰减机制 | vision.c |
| 2026-07-19 | R 电机方向 GPIO 修正 | gimbal_motor.c |
| 2026-07-30 | **PD 控制器 + 步进定位** (替代纯P+连续速度)；ROI 精准化 (收紧阈值/移除全图降级/几何约束) | vision.c, main_balance_ball.py |
