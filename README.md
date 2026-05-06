# EDC-SHEN-Car 硬件 A 底盘工程

本工程基于 TI MSPM0G3507 LaunchPad 和 CCS，用于自动识别导航小车的底盘控制部分。

当前仓库主要包含沈负责的硬件 A 底盘模块，职责包括电机驱动、底盘运动、转向、刹车、锁车、循迹控制、丢线保护、路口识别，以及编码器速度闭环。

## 当前状态

已经实现：

- 四路 TB6612 电机驱动
- 四驱轮子独立输出
- 左右差速控制
- 前进 / 后退
- 左转 / 右转
- 原地左旋 / 原地右旋
- 滑行停车
- 主动刹车
- 急停锁车
- 底盘解锁
- 循迹 PID 控制
- 丢线保护
- 原地旋转找线
- 路口 / 节点识别
- 目标节点停车
- 编码器速度闭环
- 与硬件 B 的 `track_bridge_get()` 对接，且保留本地循迹回退

尚未完全实车验证：

- 实车 PID 参数调试
- 编码器方向和速度 PID 参数
- 目标节点数量标定
- 结合编码器的终点精准停车
- 当前测试驱动板 A 路硬件异常

## 主要文件

| 文件 | 作用 |
| --- | --- |
| `empty.c` | 主函数和底盘周期调度 |
| `motor.c/.h` | TB6612 电机驱动、刹车、滑行、锁车 |
| `chassis_iface.c/.h` | 底盘对外接口、循迹控制、路径节点逻辑 |
| `track.c/.h` | 本地 8 路循迹读取回退 |
| `pid.c/.h` | PID 计算 |
| `empty.syscfg` | SysConfig 引脚配置 |
| `HARDWARE_A_INTERFACE_SYNC_REPLY.md` | 发给硬件 B 的接口对接回复 |

## 禁用引脚

以下引脚由其他模块占用，底盘电机部分不使用：

```text
PA27 PA25 PB15 PA10 PA11 PB6 PB7 PB0 PB1 PB2 PB3
```

对应用途：

```text
PA27 -> 模式选择
PA25 -> 启动按键
PB15 -> 蜂鸣器
PA10/PA11 -> 调试串口 UART0
PB6/PB7 -> 视觉 UART1
PB0/PB1/PB2/PB3 -> 灰度复用模块
```

## 电机引脚分配

当前电机控制引脚：

| 驱动信号 | MSPM0 引脚 | 说明 |
| --- | --- | --- |
| STBY | PA15 | TB6612 使能 |
| PWMA | PA28 | A 路 PWM / 使能 |
| AIN1 | PA8 | A 路方向 |
| AIN2 | PA26 | A 路方向 |
| PWMB | PB13 | B 路 PWM / 使能 |
| BIN1 | PB9 | B 路方向 |
| BIN2 | PB19 | B 路方向 |
| PWMC | PB4 | C 路 PWM / 使能 |
| CIN1 | PB17 | C 路方向 |
| CIN2 | PB18 | C 路方向 |
| PWMD | PB20 | D 路 PWM / 使能 |
| DIN1 | PB24 | D 路方向 |
| DIN2 | PA24 | D 路方向 |

轮子映射：

```text
A 路 = 左前轮
B 路 = 左后轮
C 路 = 右后轮
D 路 = 右前轮
```

电机输出接线：

```text
左前电机 -> AO1 + AO2
左后电机 -> BO1 + BO2
右后电机 -> CO1 + CO2
右前电机 -> DO1 + DO2
```

如果换正常驱动板后某个轮子方向反了，优先对调该电机的 `O1/O2` 两根输出线。只有在不能改线时，再修改 `motor.c` 中对应的电机极性。

## 底盘对外接口

接口声明在 `chassis_iface.h`。

初始化与安全控制：

```c
void chassis_init(void);
void chassis_unlock(void);
void chassis_lock(void);
void chassis_emergency_stop(void);
```

停车与刹车：

```c
void chassis_stop(void);   /* 滑行停车 */
void chassis_brake(void);  /* 主动刹车 */
```

基础运动：

```c
void chassis_drive(int forward, int turn);
void chassis_forward(int speed);
void chassis_backward(int speed);
void chassis_turn_left(int speed);
void chassis_turn_right(int speed);
void chassis_rotate_left(int speed);
void chassis_rotate_right(int speed);
```

循迹与目标控制：

```c
void chassis_set_target(chassis_target_t target);
void chassis_follow_target(int target);
void chassis_run_line(void);
void chassis_find_line(void);
```

状态查询：

```c
chassis_state_t chassis_get_state(void);
int chassis_get_node_count(void);
int chassis_is_finished(void);
```

## 与硬件 B 的对接

硬件 B 提供循迹桥接数据接口：

```c
const TrackBridgeData_t *track_bridge_get(void);
```

如果工程中存在 `track_bridge.h`，`chassis_iface.c` 会自动使用该接口。

如果当前工程暂时没有 `track_bridge.h/.c`，底盘侧会自动回退到本地：

```c
track_read()
```

这样可以保证当前工程独立编译和调试。

硬件 B 需要提供的字段：

```text
error
line_detected
all_white
center_hit
active_count
tick
stale
```

底盘侧使用规则：

```text
1. stale == true       -> 数据过期，执行保护
2. 无线 / 全白         -> 按丢线处理
3. error               -> 进入 PID 差速控制
4. active_count >= 6   -> 判断为路口 / 节点区域
```

硬件 B 的 `error` 范围为：

```text
[-100, +100]
```

底盘原 PID 使用约：

```text
[-700, +700]
```

所以当前代码中做了比例转换：

```c
line.error = bridge->error * 7;
```

## 编码器速度闭环

编码器闭环代码在 `chassis_iface.c` 中默认打开：

```c
#define CHASSIS_USE_ENCODER 1
```

编码器用于左右轮速度闭环，`chassis_drive()`、`chassis_forward()`、`chassis_rotate_left/right()` 和循迹 PID 输出都会进入速度闭环后再下发到电机。

新增的编码器调试接口：

```c
void chassis_encoder_reset(void);
void chassis_encoder_get_counts(int *e1, int *e2, int *e3, int *e4);
int chassis_encoder_get_left_count(void);
int chassis_encoder_get_right_count(void);
```

实车仍需要标定四个编码器方向、速度 PID 参数、1.5m 前进 tick 和 90 度转向 tick。

## 编译方式

在工程根目录执行：

```powershell
D:\ccs\utils\bin\gmake.exe -C Debug all
```

当前工程已确认可以编译通过。

## 运行模式

`empty.c` 中当前默认：

```c
#define RUN_MOTOR_SELF_TEST 0
#define RUN_SQUARE_TEST 0
#define RUN_ENCODER_SQUARE_TEST 0
```

表示运行正式底盘循环：

```text
SYSCFG_DL_init()
chassis_init()
chassis_set_target(CHASSIS_TARGET_C)
chassis_unlock()
周期刷新 motor_update_pwm()
周期执行 chassis_run_line()
```

如果需要单独测试电机，可以临时改成：

```c
#define RUN_MOTOR_SELF_TEST 1
```

然后重新编译烧录。

如果需要测试 1.5m 正方形，可以打开其中一个测试入口：

```c
#define RUN_SQUARE_TEST 1          /* 定时版，先粗测底盘 */
#define RUN_ENCODER_SQUARE_TEST 1  /* 编码器 tick 版，需先标定 tick */
```

两个正方形测试不要同时打开。正式联调 / 参赛时保持二者为 `0`。
