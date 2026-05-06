# 硬件 A 对接同步回复

## 0. 进一步确认问题回复

按你这边建议，下一阶段口径确认如下：

```text
1. error * 7：
这是临时兼容旧 PID 误差尺度的过渡做法，不作为长期固定接口契约。
同意改成底盘侧可配置宏，例如：

#define CHASSIS_TRACK_ERROR_SCALE 7

下一阶段先保留默认 scale=7，保证现有 PID 可继续工作；后续整车联调时可以只调这个缩放，或者再把 PID 直接改为适配 bridge 的 [-100, +100] 误差范围。

2. fallback 主路径：
正式整车集成版本的主路径固定为 track_bridge_get()。
track_read() 只作为硬件 A 单独本地编译/调试时的回退路径，不作为正式比赛集成路径。
同意增加明确编译开关，例如：

#define CHASSIS_USE_TRACK_BRIDGE 1

正式集成时设为 1；单独调试硬件 A、本地没有 track_bridge.h/.c 时才设为 0。

3. stale 动作：
stale == true 时统一执行停车保护，主策略为立即刹车。
暂不区分 DRV / NAV / ADV 模式，全部统一处理。
也就是 stale 数据不进入实时循迹 PID，不做短时维持，直接保护停车。

4. 丢线阈值与找线方向：
同意把短时丢线和长时丢线阈值参数化。
当前建议默认值沿用现有底盘逻辑：

#define CHASSIS_LOST_LINE_SHORT_TICKS 6
#define CHASSIS_LOST_LINE_LONG_TICKS  150

line_detected == false 或 all_white == true 均按疑似丢线处理。
短时丢线进入原地找线，长时丢线执行刹车保护。
找线方向采用“依据最后误差方向选择旋转方向”：
last_line_error >= 0 时右旋；
last_line_error < 0 时左旋。

5. active_count 阈值：
active_count >= 6 作为下一阶段正式默认规则，但它仍然属于需要实车标定的经验阈值。
同意做成可调宏：

#define CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD 6

6. 编码器闭环联调策略：
第一轮联调先不开编码器闭环，先跑通 track_bridge_get() + error PID + 左右差速。
也就是说下一阶段 proposal 的验收范围不需要把编码器闭环作为必过项。
编码器闭环仍由硬件 A 本地预留，后续四轮驱动、编码器方向和速度反馈稳定后再单独打开。

7. 是否需要新增主控字段：
足够，不新增字段。
当前确认 error / line_detected / all_white / center_hit / active_count / tick / stale 这 7 个字段已经足够支持下一阶段联调。
```

## 1. 当前结论

硬件 A 底盘侧已经按硬件 B 提供的桥接思路完成适配。

当前底盘侧支持两种输入来源：

- 如果工程中存在 `track_bridge.h`，底盘侧自动使用：

```c
const TrackBridgeData_t *track_bridge_get(void);
```

- 如果当前工程中暂时没有 `track_bridge.h/.c`，底盘侧自动回退到本地 `track_read()`，保证当前工程仍可独立编译和调试。

## 2. 我这边已经完成的底盘能力

硬件 A 当前已经实现：

- 四路 TB6612 电机驱动
- 四驱轮子独立控制
- 左右差速控制
- 前进 / 后退
- 左转 / 右转
- 原地左旋 / 原地右旋
- 滑行停止 `motor_coast()`
- 主动刹车 `motor_brake()`
- 底盘锁死 / 解锁
- 急停 `chassis_emergency_stop()`
- 循迹 PID 控制
- 丢线保护
- 原地旋转找线
- 路口 / 节点识别
- 目标节点停车
- 编码器速度闭环预留框架

## 3. 我这边如何消费 `TrackBridgeData_t`

底盘侧读取桥接数据后，会统一转换成底盘内部输入：

```c
typedef struct {
    int error;
    int line_detected;
    int all_white;
    int active_count;
    int stale;
} chassis_line_input_t;
```

字段对应关系：

```c
line.error = bridge->error * 7;
line.line_detected = bridge->line_detected ? 1 : 0;
line.all_white = bridge->all_white ? 1 : 0;
line.active_count = bridge->active_count;
line.stale = bridge->stale ? 1 : 0;
```

其中 `bridge->error` 的范围是 `[-100, +100]`，底盘原 PID 使用约 `[-700, +700]` 的误差尺度，所以当前做了 `* 7` 的比例转换。

## 4. 控制使用规则

底盘侧按以下优先级使用桥接数据：

1. 先判断 `stale`
   - `stale == true` 时，不把该数据作为实时循迹输入。
   - 当前策略是执行刹车保护。

2. 再判断 `line_detected / all_white`
   - `line_detected == false` 或 `all_white == true` 时，底盘侧按疑似丢线处理。
   - 短时间丢线会执行原地找线。
   - 长时间丢线会停车保护。

3. 再使用 `error`
   - `error` 进入底盘 PID。
   - PID 输出用于左右轮差速。

4. 使用 `active_count` 辅助判断路口
   - 当前 `active_count >= 6` 视为交叉 / 节点区域。

## 5. 对硬件 B 的问题回复

### 5.1 底盘控制最终消费什么？

底盘侧最终消费：

```c
track_bridge_get()->error
```

并使用 PID 差速控制，不再重复处理原始 8 路灰度。

### 5.2 底盘驱动结构是什么？

当前底盘驱动是：

```text
左右差速 PWM 控制
```

底层仍然保留四轮独立输出，接口为：

```c
void motor_set_wheels(int left_front, int left_rear, int right_rear, int right_front);
```

上层常用接口为：

```c
void motor_set_speed(int left, int right);
```

### 5.3 编码器闭环由谁实现？

编码器闭环由硬件 A 底盘侧本地实现。

当前代码中已预留：

```c
#define CHASSIS_USE_ENCODER 0
```

默认先关闭，等四轮驱动和编码器硬件稳定后改为：

```c
#define CHASSIS_USE_ENCODER 1
```

硬件 B 暂时不需要额外暴露速度状态。

### 5.4 是否需要额外丢线事件？

当前暂时不需要额外丢线事件接口。

底盘侧先使用：

```c
line_detected
all_white
stale
```

完成丢线判断与保护。

## 6. 硬件 B 需要提供的文件

后续合并时，请硬件 B 将以下文件加入工程：

```text
track_bridge.h
track_bridge.c
```

并确保头文件中提供：

```c
const TrackBridgeData_t *track_bridge_get(void);
```

以及结构体字段至少包含：

```c
error
line_detected
all_white
center_hit
active_count
tick
stale
```

## 7. 当前底盘侧对外接口

硬件 A 提供以下底盘控制接口：

```c
void chassis_init(void);
void chassis_unlock(void);
void chassis_lock(void);
void chassis_emergency_stop(void);
void chassis_brake(void);
void chassis_stop(void);

void chassis_drive(int forward, int turn);
void chassis_forward(int speed);
void chassis_backward(int speed);
void chassis_turn_left(int speed);
void chassis_turn_right(int speed);
void chassis_rotate_left(int speed);
void chassis_rotate_right(int speed);

void chassis_set_target(chassis_target_t target);
void chassis_follow_target(int target);
void chassis_run_line(void);

chassis_state_t chassis_get_state(void);
int chassis_get_node_count(void);
int chassis_is_finished(void);
```

## 8. 当前注意事项

- 当前工程中还没有 `track_bridge.h/.c`，所以本地编译时仍会回退到 `track_read()`。
- 等硬件 B 的桥接文件合入后，底盘侧会自动改用 `track_bridge_get()`。
- 当前编码器闭环默认关闭。
- 路口节点数量 `target_finish_nodes[]` 后续需要按真实赛道标定。
- PID 参数需要整车联调后继续调整。
