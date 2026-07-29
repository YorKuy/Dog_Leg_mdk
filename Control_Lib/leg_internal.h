#ifndef CONTROL_LEG_INTERNAL_H
#define CONTROL_LEG_INTERNAL_H

#include "leg.h"
#include "SMC.h"
#include "main.h"

#include <math.h>

typedef float f;
typedef uint8_t u8;

#define LEFT_LEG_MAX_MANG -0.59207F  // 左腿机械伸腿端编码器角，rad
#define LEFT_LEG_MIN_MANG -1.34095F  // 左腿图示完全收腿端编码器角，rad
#define RIGHT_LEG_MAX_MANG -2.59529F // 右腿机械伸腿端编码器角，rad
#define RIGHT_LEG_MIN_MANG -1.92694F // 右腿图示完全收腿端编码器角，rad

// 电机阻尼与输出保护。KD 和软件阻尼都增大时抑振更强，但腿会变钝且发热增加。
#define LEG_DM_MIT_KD 3.5f                       // DM MIT 内部速度阻尼；建议每次只加/减 0.2，范围不要超过驱动器 KD_MAX=5

#define LEG_PITCH_TARGET_ANGLE 0.0f      // 物理 roll 默认水平目标，degree；当前用于初始化 leg_roll_balance_target
#define LEG_PITCH_BALANCE_SIGN -1.0f     // 预留的物理 roll 对称混控符号；当前 leg_pitch_cmd=0，通常不要修改
#define LEG_PITCH_OUTPUT_LIMIT 18.0f     // 预留物理 roll 对称混控限幅，Nm；当前主路径基本不生效
#define LEG_ROLL_BALANCE_SIGN -1.0f      // 物理 pitch SMC 到镜像电机的符号；已实车验证，禁止作为增益调节
#define LEG_ROLL_KEEP_TARGET_ANGLE 1.0f  // 物理 pitch 平衡目标，degree；车体静止前后水平时按 IMU 平均值校准

// 动态补偿参数，主控制周期为 1 kHz。时间常数增大更平滑但响应更慢，减小则相反。
#define LEG_CONTROL_DT_DEFAULT 0.001f                 // 输入 dt 异常时采用的默认周期，s；必须与 1 kHz 控制周期一致
#define LEG_ACCEL_FF_ACCEL_TIME_CONSTANT 0.016f       // 加速前馈一阶滤波时间，s；振荡时可增大，补偿迟钝时可减小
#define LEG_ACCEL_FF_BRAKE_TIME_CONSTANT 0.006f       // 制动前馈滤波时间，s；减小可更快应对急停，但过小会产生冲击
#define LEG_ACCEL_FF_DECAY_TIME 0.006f                // 加速度结束后前馈衰减时间，s；增大会残留更久，可能导致急停后反弹
#define LEG_DYNAMIC_AUX_LIMIT 8.0f                    // 加速度前馈+腿速阻尼+高度保持的总限幅，Nm；增大辅助更强但会抢占 pitch 输出
#define LEG_HEIGHT_DAMPING_LIMIT 8.0f                 // 腿高速度阻尼最大输出，Nm；高速坡振荡时可小幅增加
#define LEG_HEIGHT_DAMPING_TO_ROLL_SIGN 1.0f

// 物理 roll 单侧收腿补偿。正命令收左腿，负命令收右腿；这里只调响应速度和限位。
#define LEG_ROLL_BALANCE_FILTER_TIME_CONSTANT 0.008f  // 物理 roll 力矩滤波时间，s；减小响应快，增大可减轻左右抖动
#define LEG_ROLL_RATE_FILTER_TIME_CONSTANT 0.020f     // 物理 roll 角速度滤波时间，s；角速度噪声大时增大，单边桥响应慢时减小
#define LEG_ROLL_RETRACT_SOFT_ZONE 0.05f              // 接近完全收腿端的软保护比例；增大可更早减力，但会减少限位附近 roll 权限
#define LEG_ROLL_INTEGRAL_RATE_GATE 5.0f               // 物理 roll 角速度超过该值时暂停积分，degree/s；避免动态倾斜时积分追赶
#define LEG_ROLL_INTEGRAL_ERROR_DEADZONE 0.10f         // 物理 roll 积分误差死区，degree；减小可提高精度但更易追逐 IMU 零漂
#define LEG_ROLL_INTEGRAL_LEAK_TIME 1.50f              // 进入目标死区后积分连续泄漏时间，s；禁止命中目标时突然清零

// 自动坡面支撑识别与单向高度保持。leg_enable_slope_hold=0 时以下识别参数不生效。
#define LEG_SLOPE_FORWARD_ENTER 120.0f       // 允许进入坡面候选的最小前进命令；减小更容易触发，也更容易平地误触发
#define LEG_SLOPE_FORWARD_EXIT 60.0f         // 低于该前进命令退出支撑；增大退出更早，减小可保持更久
#define LEG_SLOPE_PITCH_ENTER 2.0f           // 物理 pitch 偏差触发阈值，degree；减小更灵敏但容易被颠簸触发
#define LEG_SLOPE_STALL_ENTER 0.20f          // 四轮平均失速率阈值，0~1；减小更容易识别坡面/障碍
#define LEG_SLOPE_SMC_ENTER 6.0f             // 物理 pitch SMC 力矩阈值，Nm；减小更容易触发支撑
#define LEG_SLOPE_CONFIRM_TIME 0.032f         // 条件持续确认时间，s；增大可抗误触发但识别变慢
#define LEG_SLOPE_CANDIDATE_TIMEOUT 0.300f    // 候选状态最长等待时间，s；太短可能来不及确认，太长可能卡在候选态
#define LEG_SUPPORT_TIMEOUT 2.500f            // 支撑保持最长时间，s；增大允许长坡保持，但错误状态持续更久
#define LEG_HEIGHT_HOLD_DEADZONE 0.005f       // 归一化腿高保持死区；增大可减抖但高度波动增大
#define LEG_HEIGHT_REF_RELEASE_RATE 0.01f     // 高度参考向收腿方向的释放速度，归一化高度/s；增大释放快但保持变弱
#define LEG_HEIGHT_TO_ROLL_SIGN -1.0f         // 高度保持加到物理 pitch 轴的符号；机构方向参数，不应作为增益调整
#define LEG_HEIGHT_EMERGENCY_PITCH 8.0f       // 超过该物理 pitch 偏差时撤销高度保持，degree；减小更保守
#define LEG_HEIGHT_EMERGENCY_RATE 60.0f       // 超过该物理 pitch 角速度时撤销高度保持，degree/s；减小更早让权给 SMC

// 后轮卸载/离地缓冲。leg_enable_unload_catch=0 时以下缓冲参数不生效。
#define LEG_UNLOAD_HEIGHT_SPEED -0.25f          // 判定快速收腿的归一化腿高速度；绝对值减小会更容易触发
#define LEG_UNLOAD_TORQUE_DROP_RATIO 0.70f      // 当前反馈力矩低于峰值的该比例视为卸载；增大更容易触发
#define LEG_UNLOAD_PITCH_RATE 15.0f             // 可替代力矩下降条件的物理 pitch 角速度阈值，degree/s；减小更敏感
#define LEG_UNLOAD_CONFIRM_TIME 0.008f           // 卸载条件确认时间，s；增大可抗噪但缓冲介入更晚
#define LEG_UNLOAD_MIN_TIME 0.080f               // 卸载缓冲最短保持时间，s；增大保护更充分但动作更慢
#define LEG_UNLOAD_MAX_TIME 0.200f               // 卸载缓冲最长时间，s；增大可延长保护但影响后续收敛
#define LEG_UNLOAD_TORQUE_LIMIT 30.0f            // 卸载状态最大主动收腿力矩，Nm；减小冲击更小，但可能收腿不足
#define LEG_UNLOAD_RETRACT_SLEW_RATE 300.0f      // 卸载时继续驱动收腿的力矩变化率，Nm/s；减小更柔和
#define LEG_UNLOAD_BRAKE_SLEW_RATE 800.0f        // 卸载时制动力矩变化率，Nm/s；增大制动更及时
#define LEG_UNLOAD_DAMPING_GAIN 1.80f            // 卸载时软件关节速度阻尼；增大抑制离地后甩腿，过大会阻碍动作
#define LEG_UNLOAD_MIT_KD 4.20f                  // 卸载时 DM MIT KD；增大电机阻尼更强，禁止超过驱动器 KD_MAX=5
#define LEG_SETTLE_TIME 0.250f                   // 卸载后柔和恢复持续时间，s；增大更稳但回到正常控制更慢

// FAST/PLAYER 手动腿长串级 PID。实际 PID 增益在下方 PID_class 构造函数中，
// LEG_FAST_HEIGHT_KP/KD/UP_FF/STEP_DEADZONE 是旧版预留宏，当前没有进入计算。
#define LEG_FAST_HEIGHT_KP 242.0f                  // 旧版预留腿高 Kp，当前未使用，修改无效果
#define LEG_FAST_HEIGHT_KD 0.60f                   // 旧版预留腿高 Kd，当前未使用，修改无效果
#define LEG_FAST_HEIGHT_LIMIT 38.0f                // 角度 PID 接近该输出比例时累加前馈；增大后前馈介入更晚
#define LEG_FAST_HEIGHT_UP_FF 5.0f                 // 旧版预留上抬前馈，当前未使用，修改无效果
#define LEG_FAST_HEIGHT_STEP_DEADZONE 0.00005f     // 旧版预留目标步进死区，当前未使用，修改无效果
#define LEG_FAST_HEIGHT_TARGET_STEP_DIV 6600.0f    // 手动腿长目标步进除数；增大腿长变化更慢，减小更快
#define LEG_FAST_HEIGHT_FF_BUILD_RATE 25.0f         // 腿长误差持续增大时的前馈建立速率，Nm/s；等价旧版 0.1 Nm/4 ms
#define LEG_FAST_HEIGHT_FF_DECAY_TIME 0.10f         // 腿长前馈正常衰减时间，s；增大保持更久，过大会在换向后残留
#define LEG_FAST_HEIGHT_FF_REVERSE_TIME 0.020f      // 到达死区或误差换向后的快速衰减时间，s；仍保持连续，不突然清零
#define LEG_FAST_HEIGHT_FF_LIMIT 50.0f             // 腿长误差增长前馈限幅，Nm；增大可能超过最终输出限幅而无额外效果
#define LEG_FAST_HEIGHT_OUT_LIMIT 40.0f            // 手动腿长最终单电机力矩限幅，Nm；增大提升能力也增加结构负担
#define LEG_FAST_HEIGHT_ERROR_DEADZONE 0.010f       // 腿长角度误差死区，rad；增大可减抖但保持精度下降
#define LEG_FAST_HEIGHT_ERROR_GROW_DEADZONE 0.0003f // 判断误差正在增大的阈值，rad；减小会更频繁累加前馈
#define LEG_FAST_HEIGHT_PID_FULL_RATIO 0.90f        // PID 输出达到限幅的该比例时提前累加前馈；减小会更早介入

extern f leg_dm_mit_kd;
extern f leg_roll_cmd;
extern f leg_pitch_cmd;
extern LegControlOutput leg_output;

extern LegDynamicState leg_dynamic_state;
extern f leg_dynamic_state_time;
extern f leg_slope_confirm_time;
extern f leg_unload_confirm_time;
extern f leg_height_reference;
extern f leg_height_last;
extern f leg_height_speed_filtered;
extern f leg_feedback_torque_peak;
extern f leg_accel_ff_filtered;
extern f leg_last_forward_cmd;
extern f leg_forward_motion_sign;
extern f leg_roll_rate_filtered;
extern f leg_roll_balance_filtered;
extern f leg_roll_balance_integral;
extern u8 leg_height_initialized;
extern u8 leg_forward_initialized;

f torque_return(f u);
f leg_min(f a, f b);
f leg_max(f a, f b);
f leg_valid_dt(f dt);
f leg_decay_to_zero(f value, f dt, f time_constant);
f leg_normalized_height(f angle, f min_angle, f max_angle);
void leg_update_height(const LegControlInput *input, f dt);
void leg_enter_dynamic_state(LegDynamicState state);
void leg_update_dynamic_state(const LegControlInput *input, f smc_cmd, f dt);
f leg_accel_feedforward_update(const LegControlInput *input, f dt);
f leg_height_damping_update(void);
f leg_roll_balance_update(const LegControlInput *input, f dt);
void leg_apply_roll_retract_only(const LegControlInput *input,
                                 f roll_balance_cmd,
                                 f torque_limit);
f leg_height_hold_update(const LegControlInput *input, f dt);
void leg_sync_output(void);

f leg_limit_retract_torque(f torque, f angle_range, f limit);
f leg_unload_slew(f target, f last, f angle_range, f dt);
f leg_normal_slew(f target, f last, f motor_speed, f dt);
f leg_normal_scurve(f target, f last, f motor_speed, f dt,
                    f *rate_state, f *stage_target, u8 emergency_bypass);
f leg_apply_joint_damping_custom(f torque_cmd,
                                 MOTOR_DM *motor,
                                 f torque_limit,
                                 f damping_gain,
                                 f speed_deadzone,
                                 f angle_range);
void leg_set_balance_output_custom(const LegControlInput *input,
                                   f roll_cmd,
                                   f pitch_cmd,
                                   f torque_limit,
                                   f damping_gain,
                                   f speed_deadzone);
void leg_apply_balance_overlay(f pitch_cmd, f torque_limit);

void leg_clear_mang_pid(void);
void leg_init_height_target(const LegControlInput *input,
                            f *left_mang_cmd,
                            f *right_mang_cmd);
void leg_height_control_update(const LegControlInput *input,
                               f *left_mang_cmd,
                               f *right_mang_cmd,
                               f dt);

#endif
