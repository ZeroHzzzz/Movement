#include "PID.h"
//=======================================================DJI PID===================================================
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
void PID_init_Position(pid_type_def *pid, const float32 PID[3], float32 max_out, float32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid calculate Position
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算 位置式PID
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
float32 PID_calc_Position(pid_type_def *pid, float32 ref, float32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}

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
float32 PID_calc_DELTA(pid_type_def *pid, float32 ref, float32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
    pid->Iout = pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    pid->out += pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    // LimitMax(pid->Iout, pid->max_iout);
    return pid->out;
}

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
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}


// float Speed_Control(float32 encoder,const float pid_para[3])
// {
//     static float error_i=0;
//     static float last_error=0;
//     float error;
//     error=encoder*0.1f;  //等效/10 乘法运算更快
//     error_i += error;
//     LimitMax(error_i,200);
//     float temp_out;
//     temp_out = error*pid_para[0] + error_i*pid_para[1];
//     temp_out*=0.01f;   //等效为 /100 乘法运算更快
//     LimitMax(temp_out,9999);
//     return temp_out;
// }


/**********************************************************************************
 * @brief  PID参数设置  快速更改pid参数
 * @param pid
 * @param PID
 *********************************************************************************/
void Set_Pid_Para(pid_type_def *pid,float32 PID[3]){
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
}

/**********************************************************************************
 * @brief  PID参数设置  快速更改pid限幅
 * @param pid
 * @param max_out  //最大输出限幅
 * @param max_iout //最大积分限幅
 *********************************************************************************/
void Set_Pid_Limit(pid_type_def *pid,float32 max_out,float32 max_iout){
    pid->max_out = max_out;
    pid->max_iout = max_iout;
}


float PID_Realize_Curvature(pid_type_def *pid, float NowPoint, float TarPoint,int32 speed)
{
    //定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
    float iError, //当前误差
    Actual;   //最后得出的实际输出值
    float Kp;     //动态P
    iError = TarPoint - NowPoint; //计算当前误差
    Kp = pid->Kp;
    Actual = (float)(Kp * iError);
    return Actual;
}

/// @brief 位置式平方误差PID计算
/// @param pid 
/// @param ref 参考值
/// @param set 实际值
/// @return 
float32 PID_calc_Position_Square(pid_type_def *pid, float32 ref, float32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if(pid->error[0] >= 0){
        pid->error[0] = pid->error[0] * pid->error[0];
    }else{
        pid->error[0] = -pid->error[0] * pid->error[0];
    }
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}
/// @brief 增量式平方误差PID计算
/// @param pid 
/// @param ref 参考值
/// @param set 实际值
/// @return 
float32 PID_calc_DELTA_Square(pid_type_def *pid, float32 ref, float32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if(pid->error[0] >= 0){
        pid->error[0] = pid->error[0] * pid->error[0];
    }else{
        pid->error[0] = -pid->error[0] * pid->error[0];
    }
    pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
    pid->Iout = pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    pid->out += pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    LimitMax(pid->Iout, pid->max_iout);
    return pid->out;
}

/// @brief 位置式变速积分PID计算
/// @param pid
/// @param ref
/// @param set
/// @return
float32 PID_calc_Position_DynamicI(pid_type_def *pid, float32 ref, float32 set, float range, float iMax)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    
    // pid->Ki = (pid->error[0] > range) ? 0.0f : pid->Ki;
    
    pid->Ki = (fabsf(pid->error[0]) > range) ? 0.0f : pid->Ki * (1 - fabsf(pid->error[0]) / range);

    pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
    pid->Iout = pid->Ki * pid->error[0];
    LimitMax(pid->Iout, iMax);
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    pid->out += pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}
float PID_calc_Position_LowPassD(pid_type_def *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * (0.5f)*pid->Dbuf[0] + 0.5f*pid->Dbuf[1];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}
//TODO:微分先行
// float PID_calc_Position_Regulation(pid_type_def *pid, float ref, float set)
// {
//     if (pid == NULL)
//     {
//         return 0.0f;
//     }
//     static float pidOut[3] = {0};
//     pidOut[2] = pidOut[1];
//     pidOut[1] = pidOut[0];
//     pid->error[2] = pid->error[1];
//     pid->error[1] = pid->error[0];
//     pid->set = set;
//     pid->fdb = ref;
//     pid->error[0] = set - ref;
//     pid->Pout = pid->Kp * pid->error[0];
//     pid->Iout += pid->Ki * pid->error[0];
//     pid->Dbuf[2] = pid->Dbuf[1];
//     pid->Dbuf[1] = pid->Dbuf[0];

//     float gama = 0.12f;
//     float temp,td,c1,c2,c3;
//     gama = 0.12f;
//     td = pid->Kd / pid->Kp;
//     c1 = gama * td / (gama * td + 1);
//     c2 = (td + 1) / (gama * td + 1);
//     c3 = td / (gama * td + 1);

//     pid->Dbuf[0] = c


//     LimitMax(pid->Iout, pid->max_iout);
//     pid->out = pid->Pout + pid->Iout + pid->Dout;
//     LimitMax(pid->out, pid->max_out);
//     pidOut[0] = pid->out;
//     return pid->out;
// }