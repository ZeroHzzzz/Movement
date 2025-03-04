#ifndef TC264__PID_H
#define TC264__PID_H
#include "zf_common_typedef.h"
//====================================================================================================
// define

/*
 * @description: 输出限幅函数
 * @param input: 需要限制的值  max最大限幅
 */
#define LimitMax(input, max)           \
    {                                  \
        if ((input) > (max)) {         \
            (input) = max;             \
        } else if ((input) < -(max)) { \
            (input) = -(max);          \
        }                              \
    }
//====================================================================================================

//===================================================================================================

typedef struct PIDparam_st {
    uint32 P;
    uint32 I;
    uint32 D;
    uint32 T;
    uint32 Coeff;
    float ki;
} PIDparam_st;

typedef struct {
    float SumError;
    int32 LastError;
    int32 PrevError;
    int32 LastData;
} PIDcal_st;

//========================================================DJI
// PID========================================================

typedef struct {
    // PID 三参数
    float32 Kp;
    float32 Ki;
    float32 Kd;

    float32 max_out;   // 最大输出
    float32 max_iout;  // 最大积分输出

    float32 set;
    float32 fdb;

    float32 out;
    float32 Pout;
    float32 Iout;
    float32 Dout;
    float32 Dbuf[3];   // 微分项 0最新 1上一次 2上上次
    float32 error[3];  // 误差项 0最新 1上一次 2上上次

} pid_type_def;
/**
 * @brief          pid struct data init
 * @param[out]     pid: PID struct data point
 * @param[in]      mode: PID_POSITION: normal pid
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid max out
 * @param[in]      max_iout: pid max iout
 * @retval         none
 */
/**
 * @brief          pid struct data init
 * @param[out]     pid: PID结构数据指针
 * @param[in]      mode: PID_POSITION:普通PID
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid最大输出
 * @param[in]      max_iout: pid最大积分输出
 * @retval         none
 */
void PID_init_Position(pid_type_def* pid,
                       const float32 PID[3],
                       float32 max_out,
                       float32 max_iout);

/**
 * @brief          pid calculate
 * @param[out]     pid: PID struct data point
 * @param[in]      ref: feedback data
 * @param[in]      set: set point
 * @retval         pid out
 */
/**
 * @brief          pid计算
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @retval         pid输出
 */
float32 PID_calc_Position(pid_type_def* pid, float32 ref, float32 set);
/**
 * @brief          pid struct data init
 * @param[out]     pid: PID struct data point
 * @param[in]      mode: PID_DELTA
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid max out
 * @param[in]      max_iout: pid max iout
 * @retval         none
 */
/**
 * @brief          pid struct data init
 * @param[out]     pid: PID结构数据指针
 * @param[in]      mode: PID_DELTA  增量式PID
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid最大输出
 * @param[in]      max_iout: pid最大积分输出
 * @retval         none
 */
void PID_init_DELTA(pid_type_def* pid,
                    const float32 PID[3],
                    float32 max_out,
                    float32 max_iout);
/**
 * @brief          pid calculate PID_DELTA
 * @param[out]     pid: PID struct data point
 * @param[in]      ref: feedback data
 * @param[in]      set: set point
 * @retval         pid out
 */
/**
 * @brief          pid计算 增量式
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @retval         pid输出
 */
float32 PID_calc_DELTA(pid_type_def* pid, float32 ref, float32 set);
/**
 * @brief          pid out clear
 * @param[out]     pid: PID struct data point
 * @retval         none
 */
/**
 * @brief          pid 输出清除
 * @param[out]     pid: PID结构数据指针
 * @retval         none
 */
void PID_clear(pid_type_def* pid);

/*
 * @brief          动态修改Pid参数
 */
static inline void Change_Pid_Para(pid_type_def* pid, float32* pid_para) {
    pid->Kp = pid_para[0];
    pid->Ki = pid_para[1];
    pid->Kd = pid_para[2];
}

void Set_Pid_Limit(pid_type_def* pid, float32 max_out, float32 max_iout);
void Set_Pid_Para(pid_type_def* pid, float32 PID[3]);
//========================================================DJI
// PID========================================================

//=================================================================================================

float PID_Realize_Curvature(pid_type_def* pid,
                            float NowPiont,
                            float TarPoint,
                            int32 speed);

float Speed_Control_turn(float32 encoder, const float pid_para[3]);

float PID_calc_Position_LowPassD(pid_type_def* pid, float ref, float set);
float32 PID_calc_Position_DynamicI(pid_type_def* pid,
                                   float32 ref,
                                   float32 set,
                                   float range,
                                   float iMax);
#endif
