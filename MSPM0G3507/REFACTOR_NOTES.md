# 控制框架重构文档

> 第一次重构: 2026-07-23 — 轻量 ISR + 开环/闭环可切换 + 全定点整数运算  
> 第二次重构: 2026-07-24 — 删除运行时模式切换 + 开环/闭环独立文件 + `#ifdef` 编译时选择  
> 第三次重构: 2026-07-25 — merge main 分支 + 适配 JY61P 陀螺仪角度闭环 + 双模式共存

---

## 1. 重构动机

### 第一次重构 (2026-07-23)

| 问题 | 严重程度 | 说明 |
|------|:--------:|------|
| ISR 中软浮点运算 | 致命 | M0+ 无 FPU，每次 ISR ~14 次 `__aeabi_fmul/div`，耗时 200-500μs |
| 开环/闭环互相冲突 | 致命 | `trace_motor()` 直接写 PWM，紧接着 `motor_PID(target=0)` 覆盖 |
| 竞态条件 | 致命 | `trace_motor()` 同时被 ISR 和 `while(1)` 主循环调用 |
| 控制周期不可调 | | PID_T=20ms 硬编码，降低周期会放大浮点开销 |

### 第二次重构 (2026-07-24)

| 问题 | 说明 |
|------|------|
| 运行时模式切换冗余 | 开环和闭环共用 `control.c`，通过 `control_mode_t` 枚举在 ISR 中分支判断 |
| 耦合度高 | 开环常量和闭环 PID 状态混在同一文件，修改任一模式需触碰整个文件 |
| 按键逻辑复杂 | KEY_1/KEY_2 在 3 个 status 间循环（0→1→2），用户需记住当前处于哪种模式 |

### 第三次重构 (2026-07-25)

| 问题 | 说明 |
|------|------|
| main 分支代码合并 | main 已用 JY61P 陀螺仪 + 角度闭环替代了 MPU6050，需要合并但不破坏本分支架构 |
| 旧 MPU 文件删除 | `mpu_port.c` 被删导致 `sys_tick_ms` 定义丢失 |
| 角度闭环接口适配 | `angle_control.c` 依赖的 `Motor_SetSpeed()` 在本分支不存在 |
| 双模式共存 | 循迹 ISR 和角度闭环需要互斥，不能同时写电机 |

---

## 2. 文件变更清单

### 第三次重构 (2026-07-25)

| 文件 | 操作 | 说明 |
|------|:--:|------|
| `User/jy61p_port.h` | 来自 main | JY61P 姿态传感器驱动，自包含无须改动 |
| `User/jy61p_port.c` | 来自 main | UART 解析 + 浮点角度转换 |
| `User/angle_control.h` | 来自 main | 角度闭环 PID 接口 |
| `User/angle_control.c` | 来自 main | 位置式 PID，依赖 `Motor_SetSpeed()` 和 `JY61P_Get_Yaw()` |
| `User/motor.h` | 修改 | 新增 `Motor_SetSpeed()`、`angle_ctrl_active`、`last_angle_ctrl_ms`、`sys_tick_ms` 声明 |
| `User/motor.c` | 修改 | 新增 `Motor_SetSpeed()` 实现（用本分支 `motor_set_direction/duty`） |
| `User/control.c` | 修改 | 新增 `angle_ctrl_active` 接管检查 + 100ms 超时自动恢复循迹 |
| `User/control_closed.h` | 修改 | `SPEED_GAIN_Q8_8`: 7680(30.0) → 2560(10.0)，修复差速过激导致转圈 |
| `User/trace.c` | 修改 | 修正传感器布局注释：X2 为最左，X1 为内侧（匹配物理布局 X2-X1-X3-X4） |
| `main.c` | 重写 | 集成 JY61P 初始化 + 角度闭环循环 + OLED 显示 + 模式切换（HOLD→STRAIGHT） |
| `Debug/makefile` | 修改 | `inv_mpu/mpu_port` → `angle_control/jy61p_port` |
| `Debug/.../subdir_vars.mk` | 修改 | 同上 |

删除文件（来自 main 合并）：
| 文件 | 原因 |
|------|------|
| `User/mpu_port.c/h` | 已被 `jy61p_port.c/h` 替代 |
| `User/inv_mpu.c/h` | MPU6050 DMP 驱动，不再使用 |
| `User/inv_mpu_dmp_motion_driver.c/h` | 同上 |
| `User/dmpKey.h`、`User/dmpmap.h` | 同上 |

### 第二次重构 (2026-07-24)

| 文件 | 操作 | 说明 |
|------|:--:|------|
| `User/control_config.h` | **新建** | `#define CONTROL_OPEN_LOOP` / `CONTROL_CLOSED_LOOP` 二选一开关 |
| `User/control.h` | **重写** | 精简为只声明 `control_init()` |
| `User/control.c` | **重写** | ISR 入口 + 传感器读取 + 特殊状态；`#ifdef` 引入对应头文件 |
| `User/control_open.h` | **新建** | 开环常量 + 4 个 variant hook 函数声明 |
| `User/control_open.c` | **新建** | 开环实现，整段在 `#ifdef CONTROL_OPEN_LOOP` 内（~30 行） |
| `User/control_closed.h` | **新建** | 闭环常量 + 4 个 variant hook 函数声明 |
| `User/control_closed.c` | **新建** | 闭环实现 + 完整 PID，整段在 `#ifdef CONTROL_CLOSED_LOOP` 内（~130 行） |
| `User/interrupt.c` | 修改 | KEY_1/KEY_2 改为 `status = !status`（启停切换） |
| `User/key.h` | 修改 | 删除未使用的 `STATUS_TRACE` 宏 |
| `main.c` | 修改 | 删除 `last_mode`/`desired_mode`/switch-case；OLED 简化为 `S:%d` |

### 第一次重构 (2026-07-23)

| 文件 | 操作 | 说明 |
|------|:--:|------|
| `User/control.h` | **新建** | 控制模式枚举 + API 声明 |
| `User/control.c` | **新建** | 定点 PID、模式管理、ISR 入口、control_update() |
| `User/trace.h` | 重写 | 移除 `target_speed_A/B`、`trace_motor()`；新增传感器 API |
| `User/trace.c` | 重写 | 移除控制逻辑和 `DUTY_*` 宏；保留纯传感器读取 |
| `User/motor.h` | 修改 | 移除 `calculate_speed()`、`motor_PID()`；新增 `motor_read_encoder()` |
| `User/motor.c` | 修改 | 移除全部 PID 代码和 `PID_INST_IRQHandler`；新增定点编码器读取 |
| `main.c` | 修改 | 移除 `trace_motor()` 调用；status 自动映射控制模式 |

---

## 3. 架构

```
┌──────────────────────────────────────────────────────────────┐
│                   应用层 (main.c)                            │
│  初始化 → 启动定时器 → while(1) 只刷 OLED                     │
│  之后所有控制由硬件定时器自动驱动，main 不再碰电机               │
│                                                               │
│  双模式切换：注释/取消注释循迹初始化区域即可                      │
│    循迹模式：control_init + PID定时器 + NVIC                  │
│    角度闭环：JY61P_Init + Angle_Control_Update 主循环          │
└────────────────────┬─────────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────────┐
│              控制层 (control.c)                               │
│                                                               │
│  #include "control_config.h"  ← 切换 CONTROL_OPEN/CLOSED_LOOP │
│  #ifdef CONTROL_OPEN_LOOP                                     │
│    #include "control_open.h"                                  │
│  #else                                                        │
│    #include "control_closed.h"                                │
│  #endif                                                       │
│                                                               │
│  PID_INST_IRQHandler()  ← 硬件定时器每 20ms 触发               │
│    └── control_update()  ← 传感器读取 + 特殊状态（共享骨架）     │
│          │                                                     │
│          ├─ 0. angle_ctrl_active? → 超时>100ms 恢复，否则 return│
│          ├─ 1. active==0        → STOP + variant_reset()       │
│          ├─ 2. inner || active==4 → control_straight_duty()    │
│          └─ 3. normal            → control_track_duty(error)   │
│                                                               │
│  四个 variant hook 在链接时解析（两个 .c 只有一个产出代码）      │
└──┬────────────────────────────────────────────────────────────┘
   │
   │  ┌───────────────────────────────────────────────┐
   │  │    Variant 层 — 两个 .c 都编译，只有一个有代码    │
   │  │                                                │
   │  │  control_open.c          control_closed.c      │
   │  │  ─────────────           ────────────────       │
   │  │  #ifdef OPEN_LOOP        #ifdef CLOSED_LOOP    │
   │  │    ... 开环实现 ...         ... 闭环 + PID ...   │
   │  │  #endif                  #endif                │
   │  │                                                │
   │  │  未选中的 #ifdef 不命中 → 编译出空 .o            │
   │  └───────────────────────────────────────────────┘
   │                          │
   │ trace_get_error()        │ motor_read_encoder()
   │ trace_read()             │ motor_set_duty()
   │                          │ motor_set_direction()
┌──▼──────────────────┐  ┌──▼──────────────────────────────────┐
│  传感器层 (trace.c)   │  │       驱动层 (motor.c)               │
│                      │  │                                      │
│  trace_read()        │  │  motor_set_duty()      写 PWM 比较器  │
│  trace_get_error()   │  │  motor_set_direction()  写 GPIO 方向  │
│  trace_get_active()  │  │  motor_read_encoder()   读+清零编码器  │
│  trace_inner_both_   │  │  limit_duty()           PWM 钳位     │
│    line()            │  │  Motor_SetSpeed()       角度闭环接口  │
│                      │  │                                      │
│  4 路红外 GPIO 读取   │  │  [编码器脉冲由 interrupt.c 的          │
│  布局: X2 X1 X3 X4   │  │   GROUP1_IRQHandler 累加]            │
│   (左→右)            │  │                                      │
└──────────────────────┘  └──────────────────────────────────────┘
```

### 角度闭环独立模块（编译但不一定启用）

```
┌──────────────────────────────────────────────────────────────┐
│  角度闭环模块 (main 分支引入，main.c 中按需启用)                │
│                                                               │
│  jy61p_port.c/h          angle_control.c/h                    │
│  ─────────────           ─────────────────                    │
│  JY61P UART 解析         位置式 PID (Yaw 角度)                 │
│  JY61P_Get_Yaw()   ──►   Angle_Control_Update()               │
│                                   │                            │
│                            Motor_SetSpeed(L,R)                 │
│                                   │                            │
│              ┌────────────────────┼────────────────────┐      │
│              │  motor_set_direction + motor_set_duty    │      │
│              │  angle_ctrl_active = 1                   │      │
│              │  → control_update() ISR 跳过循迹         │      │
│              └──────────────────────────────────────────┘      │
└──────────────────────────────────────────────────────────────┘
```

---

## 4. Variant Hook 接口

四个 hook 函数分别在 `control_open.h` / `control_closed.h` 中声明，在 `control_open.c` / `control_closed.c` 中实现。

```c
void control_variant_init(void);           // 模式特定初始化
void control_variant_reset(void);          // 停车时重置内部状态
void control_straight_duty(void);          // inner/lost 直行占空比
void control_track_duty(int8_t error);     // 正常循迹占空比
```

### 数据流（一次控制周期 = 20ms）

```
PID_INST 定时器溢出
  │
  └─► PID_INST_IRQHandler()
        │
        └─► control_update()
              │
              ├─ 0. angle_ctrl_active==1 且未超时 → return（角度闭环接管中）
              │
              ├─ 1. trace_read()            读 4 路 GPIO → trace_data[4]
              ├─ 2. trace_get_error()       质心误差 [-10, +10]
              ├─ 3. trace_get_active()      白区传感器计数 [0, 4]
              ├─ 4. trace_inner_both_line() X1 和 X3 是否同时在线上（居中）
              │
              ├─ 5. active == 0  → STOP + duty=0 + variant_reset()
              │
              ├─ 6. inner || active==4 → FORWARD
              │       └── control_straight_duty()
              │             ├── [开环] duty = 1300+20 / 1300
              │             └── [闭环] PID → BASE_SPEED(100mm/s)
              │
              └─ 7. normal → FORWARD
                      └── control_track_duty(error)
                            ├── [开环] duty = 1300 ± error × 300
                            │              → motor_set_duty()
                            │
                            └── [闭环] target = 100 ± error × 10 (mm/s)
                                        → motor_read_encoder() ×2
                                        → pid_update() ×2
                                        → duty_Q8_8 累加 + 钳位
                                        → motor_set_duty()
```

---

## 5. 模式切换

### 循迹开环/闭环切换

在 `User/control_config.h` 中修改一行宏：

```c
#define CONTROL_OPEN_LOOP       // 开环
// #define CONTROL_CLOSED_LOOP

// #define CONTROL_OPEN_LOOP
#define CONTROL_CLOSED_LOOP     // 闭环
```

修改后重新编译。两个 `.c` 都参与编译，未被选中的 `#ifdef` 不命中，产出空 `.o`，不产生符号冲突。构建系统无需修改。

### 循迹 / 角度闭环切换

在 `main.c` 中注释或取消注释对应的初始化区域：

**循迹模式**：
```c
motor_init(MOTOR_L);
motor_init(MOTOR_R);
control_init();
DL_Timer_startCounter(PID_INST);
NVIC_EnableIRQ(PID_INST_INT_IRQN);
// JY61P + 角度闭环 while(1) 注释掉
```

**角度闭环模式**：
```c
motor_init(MOTOR_L);
motor_init(MOTOR_R);
// control_init / 定时器 / NVIC 注释掉
JY61P_Init();
delay_ms(2000);
// 角度闭环 while(1) 启用
```

---

## 6. 定点数约定

Cortex-M0+ 无硬件 FPU 和除法器。所有循迹计算用 `int32_t`，通过缩放实现小数精度。

| 物理量 | 内部表示 | 缩放因子 | 范围 |
|--------|:------:|:------:|------|
| 速度 | Q8.8 | ×256 | ±32767 mm/s |
| PID 增益 | ×256 | kp=0.5 → 128 | ±128 |
| PWM 占空比 (累加器) | Q8.8 | ×256 | 0 ~ 4000×256 |
| 传感器误差 | 原生 | ×1 | -10 ~ +10 |

> 注意：角度闭环模块（`angle_control.c`、`jy61p_port.c`）使用 `float` 软浮点，运行在主循环上下文（非 ISR），通过 `__disable_irq()` 保护浮点运算的原子性。

---

## 7. 调参指南

### 开环参数 (`control_open.h`)

```c
#define DUTY_STRAIGHT  1300    // 直行基准占空比 (0~4000)
#define DUTY_GAIN       300    // 误差增益 (PWM per error unit)
#define DUTY_BIAS        20    // 左轮偏置 (补偿硬件不对称)
```

| 现象 | 操作 |
|------|------|
| 跑偏跟不上线 | DUTY_GAIN (300→400) |
| 车速太快冲过头 | DUTY_STRAIGHT (1300→1000) |
| 转弯剧烈摇摆 | DUTY_GAIN (300→200) |
| 直行偏右 | DUTY_BIAS (20→40) |
| 直行偏左 | DUTY_BIAS (20→0) |

### 闭环参数 (`control_closed.h`)

```c
#define BASE_SPEED_Q8_8   25600   // 目标速度 Q8.8 (100.0 mm/s)
#define SPEED_GAIN_Q8_8    2560   // 误差→速度映射 Q8.8 (10.0)
#define KP_Q8_8            128    // PID 比例增益 Q8.8 (0.5)
#define KI_Q8_8            102    // PID 积分增益 Q8.8 (0.4)
```

按顺序调：

| 步骤 | 参数 | 方法 |
|:--:|------|------|
| 1 | `KP_Q8_8` | `KI_Q8_8=0`，逐步加大 KP 至直线上轻微震荡，回退到 70% |
| 2 | `KI_Q8_8` | 从 0 逐步加大，至稳态误差消失且不震荡 |
| 3 | `SPEED_GAIN_Q8_8` | 弯道差速不够则加大 |
| 4 | `BASE_SPEED_Q8_8` | 确定巡航速度 |

### 角度闭环参数 (`angle_control.c`)

```c
float Angle_Kp = 40.0f;    // 比例增益
float Angle_Ki = 5.0f;     // 积分增益
float Angle_Kd = 3.0f;     // 微分增益
```

### 硬件改动时需重算

| 改动 | 文件 | 参数 |
|------|------|------|
| 编码器 PPR | `motor.h` | `MOTOR_BIANMAQI` |
| 轮径 | `motor.h` | `MOTOR_WHEEL_D` |
| 以上任意一项 | `motor.c` | 重算 `SPEED_FACTOR_Q8_8` |
| 控制周期 | `control_closed.c` | `PID_PERIOD_MS`，重算 `SPEED_FACTOR_Q8_8` |

---

## 8. 注意事项

1. **编码器读清零竞态**: `motor_read_encoder()` 在读取和清零之间可能丢失至多 1 个脉冲 (~0.05mm 位移)，M0+ 无原子 RMW 指令，此误差可接受。
2. **PID 占空比累加器** (仅闭环): `duty_L_Q8_8` / `duty_R_Q8_8` 启动时初始化为 `INIT_DUTY_Q8_8`（1300×256）而非 0，避免冷启动顿挫。
3. **Variant 编译安全**: 四个 hook 函数由两个 `.c` 提供，`control_config.h` 中的 `#ifdef` 决定哪个产出代码。决不可同时定义两个宏，否则链接报重复符号。
4. **构建系统**: 第三次重构修改了 `makefile` / `subdir_vars.mk`（MPU → angle_control/jy61p_port）。若用 CCS/SysConfig 重新生成项目文件，需重新手动调整。
5. **`SPEED_FACTOR_Q8_8`**: 仅在 `motor.c` 中定义。
6. **`sys_tick_ms`**: 在 `main.c` 中定义（`volatile uint32_t sys_tick_ms = 0`），多个文件通过 `motor.h` 中的 `extern` 声明引用。
7. **角度闭环软浮点**: `angle_control.c` 和 `jy61p_port.c` 使用 `float`，在 M0+ 上通过软浮点库实现。主循环中调用 `Angle_Control_Update()` 时用 `__disable_irq()` 保护，防止 ISR 中的 `control_update()` 浮点重入（虽然当前循迹 ISR 不使用浮点，但保留保护机制）。
8. **循迹/角度闭环互斥**: `Motor_SetSpeed()` 设置 `angle_ctrl_active=1`，`control_update()` ISR 检查此标志并跳过循迹。若角度闭环超过 100ms 未调用 `Motor_SetSpeed()`，ISR 自动恢复循迹。
