//
// Created by R0602 on 26-7-10.
//
#include "Shoot_Ctrl.h"
#include "System_State.h"
#include "Robot_Config.h"
#include "Robot_Cmd.h"
#include "Horizon_MATH.h"
#include "VT13.h"
#include "DBUS.h"
#include "Referee.h"
#include "Vofa.h"
//TODO:发射部分待完成
static Shoot_Ctrl_Block_t shoot_ctrl;
/**
 * @brief 发射机构控制初始化
 * @param MOTOR 发射机构电机总结构体指针
 * @return uint8_t 初始化状态
 */
uint8_t Shoot_Control_Init(void)
{
    shoot_ctrl.motor_ratio = 36.0f;//拨盘电机减速比
    shoot_ctrl.feed_ratio = 2.5f;//拨盘齿轮减速比
    shoot_ctrl.slot_num = 9.0f;//拨盘一圈弹槽数量
    shoot_ctrl.use_smoothing = 0;//不开启平滑连发
    shoot_ctrl.dir_sign = -1;//拨盘旋向为反
    shoot_ctrl.Feeder_Count.target_freq = 18;//目标弹频

    shoot_ctrl.Lfire_speed = -6500.0f;//左摩擦轮目标转速
    shoot_ctrl.Rfire_speed = 6500.0f;//右摩擦轮目标转速

    shoot_ctrl.Counts_Shoot=shoot_ctrl.motor_ratio * shoot_ctrl.feed_ratio * 8192.0f / shoot_ctrl.slot_num;//计算拨盘每一发需要的编码器值
    // 拨盘电机PID参数初始化
    float PID_Bmotor_P[3] = {0.23f,   0.0f,  0.0f};
    PID_Init(&shoot_ctrl.Bmotor_P, 20000.0f, 30.0f, PID_Bmotor_P,
        0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    float PID_Bmotor_S[3] = {15.0f,   0.0f,   0.0f};
    PID_Init(&shoot_ctrl.Bmotor_S, 8000.0f, 2.0f, PID_Bmotor_S,
             0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    //两个摩擦轮 PID参数初始化
    float PID_Lfire_S[3] = {10.0f,   0.0f,  0.0f};
    PID_Init(&shoot_ctrl.Lfire_S, 16000.0f, 1000.0f, PID_Lfire_S,
        0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    float PID_Rfire_S[3] = {10.0f,   0.0f,   0.0f};
    PID_Init(&shoot_ctrl.Rfire_S, 16000.0f, 1000.0f, PID_Rfire_S,
             0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    //向系统下发发射当前状态，准备中
    System_State_Report(ID_SHOOT, STATUS_PREPARING);
    return 1;
}

/**
 * @brief 连发控制
 */
void Smooth_Shoot_Control(void)
{
    //连发
    shoot_ctrl.Feeder_Count.target_freq = MATH_Limit_float(shoot_ctrl.Feeder_Count.target_freq, 0.0f, 25.0f);
    if (shoot_cmd.trigger_auto==1&&VT13.Remote.trigger==1)
    {
        shoot_ctrl.use_smoothing=1;
    }
    else if (shoot_cmd.trigger_auto ==1 && VT13.Remote.trigger  ==0)
    {
        shoot_ctrl.use_smoothing=0;
    }
}

/**
 * @brief 卡弹检测TODO
 */
void Automatic_Shoot_Control(const Shoot_Motor_Group_t *g_motor)
{

}
/**
 * @brief 发射总控制任务
 */
void Shoot_Control_Task(const Shoot_Motor_Group_t *g_motor, float dt)
{
    // 空指针保护
    if (g_motor == NULL ) {
        System_State_Report(ID_SHOOT, STATUS_ERROR);
        return;
    }
    // 状态汇报
    if (!Is_Group_Online(SHOOT)) {
        System_State_Report(ID_SHOOT, STATUS_LOST);
    }
    else{System_State_Report(ID_SHOOT, STATUS_RUN);}
    // 判断系统状态
    bool is_system_locked = (sys_state.global_mode == GLOBAL_SAFE_LOCK ||
                             sys_state.global_mode == GLOBAL_STANDBY ||
                             sys_state.global_mode == GLOBAL_INIT_STAGE);
    if (shoot_cmd.mode == SHOOT_CMD_SAFE || is_system_locked)
    {
        // 清空PID
        PID_Clear(&shoot_ctrl.Bmotor_P);
        PID_Clear(&shoot_ctrl.Bmotor_S);
        shoot_ctrl.Lfire_speed = 0.0f;
        shoot_ctrl.Rfire_speed = 0.0f;
        shoot_ctrl.Bmotor_P.Ref = g_motor->DJI_2006_bo.Angle_Infinite;
    }
    static uint32_t last_shot_time = 0;
    static bool is_init =false;
    static float smooth_ref = 0.0f;
    // 发弹检测
    bool bullet_fired = Update_Shoot_Det_Dynamic(
        g_motor->DJI_3508_L.Speed_now,
        g_motor->DJI_3508_R.Speed_now,
               dt, &shoot_ctrl.Det_Count);

    // 根据剩余热量计算动态弹频
    //shoot_ctrl.Feeder_Count.target_freq = Heat_Freq_Ctrl(0.4f,shoot_cmd.heat_max,shoot_cmd.heat_now,shoot_cmd.cool,bullet_fired,dt,18,10);

//包含摩擦轮是否开启
    if (shoot_cmd.mode == SHOOT_CMD_READY)
    {
        shoot_ctrl.Lfire_speed = 0.0f;
        shoot_ctrl.Rfire_speed = 0.0f;
        shoot_ctrl.Bmotor_P.Ref = smooth_ref = g_motor->DJI_2006_bo.Angle_Infinite;
        shoot_ctrl.Feeder_Count.target_pos_cnt = (int32_t)ceilf(smooth_ref / (shoot_ctrl.Counts_Shoot) - 0.1f);
    }
    if (shoot_cmd.mode == SHOOT_CMD_RUN || shoot_cmd.mode == SHOOT_CMD_FIRE) {
        shoot_ctrl.Lfire_speed = -6500.0f;//左摩擦轮目标转速
        shoot_ctrl.Rfire_speed = 6500.0f;//右摩擦轮目标转速
        if (!is_init)
        {
            smooth_ref = g_motor->DJI_2006_bo.Angle_Infinite;
            shoot_ctrl.Feeder_Count.target_pos_cnt=(int32_t)ceilf(smooth_ref / shoot_ctrl.Counts_Shoot - 0.1f);
            is_init = true;
        }
        uint32_t now = HAL_GetTick();
        float interval = 1000.0f / shoot_ctrl.Feeder_Count.target_freq;
        //连发模式同时已经开启摩擦轮
        if (shoot_cmd.mode == SHOOT_CMD_FIRE)
        {
            Smooth_Shoot_Control();
            if (now - last_shot_time >= (uint32_t)interval) {
                float current_target_angle = (float)shoot_ctrl.Feeder_Count.target_pos_cnt * shoot_ctrl.Counts_Shoot * (float)shoot_ctrl.dir_sign;
                float angle_error = fabsf(current_target_angle - g_motor->DJI_2006_bo.Angle_Infinite);
                // 只有当误差小于1.1发弹丸的角度时，才允许下发新的发弹指令
                if (angle_error < (1.5f * shoot_ctrl.Counts_Shoot)) {
                    shoot_ctrl.Feeder_Count.target_pos_cnt ++;////////
                }
                else {
                    System_State_Report(ID_SHOOT, STATUS_ERROR);
                    shoot_ctrl.Bmotor_P.Ref = smooth_ref = g_motor->DJI_2006_bo.Angle_Infinite;
                    shoot_ctrl.Feeder_Count.target_pos_cnt = (int32_t)ceilf(smooth_ref / (shoot_ctrl.Counts_Shoot) - 0.1f);
                    DJI_Motor_Send(&hcan1,0x200,0,0,0,0);
                }
                last_shot_time = now;
            }
        }
        // 统一目标计算与运动平滑控制
        float final_target = (float)shoot_ctrl.Feeder_Count.target_pos_cnt * shoot_ctrl.Counts_Shoot * (float)shoot_ctrl.dir_sign;
        //连发
        if (shoot_ctrl.use_smoothing ==1)//使用平滑模式
        {
            float step = (shoot_ctrl.Feeder_Count.target_freq * shoot_ctrl.Counts_Shoot) * dt;

            if (smooth_ref > final_target)
            {
                smooth_ref -= step;
                if (smooth_ref < final_target) smooth_ref = final_target;
            } else if (smooth_ref < final_target)
            {
                smooth_ref += step;
                if (smooth_ref > final_target) smooth_ref = final_target;
            }
        }
        else if(shoot_ctrl.use_smoothing ==0)
        {
            smooth_ref = final_target;
        }
        else
        {
            smooth_ref = g_motor->DJI_2006_bo.Angle_Infinite;;
        }
    }
    shoot_ctrl.Lfire_S.Ref=shoot_ctrl.Lfire_speed;
    shoot_ctrl.Rfire_S.Ref=shoot_ctrl.Rfire_speed;
    shoot_ctrl.Bmotor_P.Ref=smooth_ref;

    PID_Calculate(&shoot_ctrl.Lfire_S, g_motor->DJI_3508_L.Speed_now, shoot_ctrl.Lfire_S.Ref);
    PID_Calculate(&shoot_ctrl.Rfire_S, g_motor->DJI_3508_R.Speed_now, shoot_ctrl.Rfire_S.Ref);
    PID_Calculate(&shoot_ctrl.Bmotor_P, g_motor->DJI_2006_bo.Angle_Infinite, shoot_ctrl.Bmotor_P.Ref);
    PID_Calculate(&shoot_ctrl.Bmotor_S, g_motor->DJI_2006_bo.Speed_now, shoot_ctrl.Bmotor_P .Output);

    DJI_Motor_Send(&hcan2,0x200,shoot_ctrl.Lfire_S.Output,shoot_ctrl.Rfire_S.Output,0,0);
    DJI_Motor_Send(&hcan1,0x200,0,0,shoot_ctrl.Bmotor_S.Output,0 );
    VOFA_JustFloat(&huart1, 5, shoot_cmd.heat_max,shoot_cmd.heat_now,shoot_cmd.cool,bullet_fired);
}
/**
 * @brief  动态 dt 射击检测函数
 * @param  speed1: 摩擦轮1转速
 * @param  speed2: 摩擦轮2转速
 * @param  dt: 当前帧与上一帧的真实时间差（s）
 * @param  det: 检测器结构体指针
 * @retval bool: 是否成功检测到一发子弹射出
 */
bool Update_Shoot_Det_Dynamic(float speed1, float speed2, float dt, ShootDet_t *det) {
    // 防止 dt <= 0 导致除以零或逻辑错误
    if (dt <= 0.0001f) dt = BASE_DT;
    det->last_cnt = det->cnt;
    float val = (fabsf(speed1) + fabsf(speed2)) / 2.0f;
    // 初始化
    if (!det->init) {
        det->base = val;
        det->last_val = val;
        det->max_drop_in_round = 0;
        det->cool_down_timer = 0.0f;
        det->t_out_timer = 0.0f;
        det->init = true;
        return false;
    }
    // 斜率计算 (每秒跌落的 RPM)
    float slope_per_sec = (det->last_val - val) / dt;
    det->last_val = val;
    // 根据dt动态调整低通滤波系数
    float alpha_up = (K_UP_BASE * dt) / (BASE_DT + (K_UP_BASE - 1.0f) * (BASE_DT - dt));
    float alpha_dn = (K_DN_BASE * dt) / (BASE_DT + (K_DN_BASE - 1.0f) * (BASE_DT - dt));
    // 限幅
    if (alpha_up > 1.0f) alpha_up = 1.0f; else if (alpha_up < 0.0f) alpha_up = 0.0f;
    if (alpha_dn > 1.0f) alpha_dn = 1.0f; else if (alpha_dn < 0.0f) alpha_dn = 0.0f;
    // 更新基准线
    if (val > det->base) {
        det->base = (alpha_up * val) + (1.0f - alpha_up) * det->base;
    } else {
        det->base = (alpha_dn * val) + (1.0f - alpha_dn) * det->base;
    }
    float drop = det->base - val;
    bool shoot_done = false;
    // 冷却定时器递减
    if (det->cool_down_timer > 0.0f) {
        det->cool_down_timer -= dt;
        det->armed = false;
        return false;
    }
    // 检测状态机
    if (!det->armed) {
        // 未触发状态：检测是否满足射击瞬间跌落条件
        if (drop > TH_FIRE && drop < TH_FIRE_MAX && slope_per_sec > MIN_SLOPE_PER_SEC && val > 4500.0f) {
            det->armed = true;
            det->max_drop_in_round = drop;
            det->t_out_timer = 0.0f; // 清空超时计时
        }
    } else {
        // 已触发状态：累加超时时间
        det->t_out_timer += dt;
        if (drop > det->max_drop_in_round) {
            det->max_drop_in_round = drop;
        }
        // 判定转速是否开始回升
        bool condition_relative = (drop < det->max_drop_in_round * (1.0f - RELATIVE_RECOVER));
        bool condition_absolute = (drop < TH_RST_SAFE);
        if (condition_relative || condition_absolute) {
            // 成功检测到射击完成
            det->armed = false;
            det->cnt++;
            det->cool_down_timer = COOLDOWN_DURATION; // 加载冷却时间
            det->max_drop_in_round = 0;
            shoot_done = true;
        }
        else if (det->t_out_timer >= TIMEOUT_DURATION) {
            // 超时未回升，强行复位
            det->armed = false;
            det->max_drop_in_round = 0;
        }
    }
    return shoot_done;
}

/**
 * @brief  枪口热量控制
 * @note   通过摩擦轮掉速检测与裁判系统数据融合计算实时热量，并引入安全余量动态调整最大容许发弹频率
 * @param  kp           发弹频率控制的比例系数
 * @param  heat_max     最大热量上限
 * @param  heat_now     当前裁判系统反馈热量
 * @param  cool         枪口冷却速度
 * @param  is_shot      是否检测到弹丸射出
 * @param  dt           当前帧与上一帧的运行时间差 (单位: 秒)
 * @param  max_freq     最大允许发弹频率 (单位: Hz)
 * @param  remain_heat  剩余热量
 * @return float        经过热量限制及幅值控制后的最终目标发弹频率 (单位: Hz)
 */
float Heat_Freq_Ctrl(float kp, float heat_max,float heat_now,float cool, bool is_shot, float dt, float max_freq, float remain_heat) {
    static float internal_heat = 0.0f;
    static float target_freq = 0.0f;
    // 累计热量计算
    if (is_shot) {
        internal_heat += 10.0f;
    }
    // 减去枪口冷却
    internal_heat -= cool * dt;
    // 与裁判系统数据融合校准
    if (internal_heat < heat_now) {
        internal_heat = heat_now;
    }
    // 根据剩余热量动态计算安全射频
    target_freq = kp * (heat_max - internal_heat - remain_heat);
    // 限制最大射频上限
    return MATH_Limit_float(target_freq, 0.0f, max_freq);
}