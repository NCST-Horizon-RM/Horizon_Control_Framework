//
// Created by CaoKangqi on 2026/1/19.
//
#include "Classic_Control.h"
#include "Horizon_MATH.h"

/******************************** FUZZY PID **********************************/
static float FuzzyRuleKpRAW[7][7] = {
    {PB, PB, PM, PM, PS, ZE, ZE},
    {PB, PB, PM, PS, PS, ZE, NS},
    {PM, PM, PM, PS, ZE, NS, NS},
    {PM, PM, PS, ZE, NS, NM, NM},
    {PS, PS, ZE, NS, NS, NM, NM},
    {PS, ZE, NS, NM, NM, NM, NB},
    {ZE, ZE, NM, NM, NM, NB, NB}};

static float FuzzyRuleKiRAW[7][7] = {
    {NB, NB, NM, NM, NS, ZE, ZE},
    {NB, NB, NM, NS, NS, ZE, ZE},
    {NB, NM, NS, NS, ZE, PS, PS},
    {NM, NM, NS, ZE, PS, PM, PM},
    {NM, NS, ZE, PS, PS, PM, PB},
    {ZE, ZE, PS, PS, PM, PB, PB},
    {ZE, ZE, PS, PM, PM, PB, PB}};

static float FuzzyRuleKdRAW[7][7] = {
    {PS, NS, NB, NB, NB, NM, PS},
    {PS, NS, NB, NM, NM, NS, ZE},
    {ZE, NS, NM, NM, NS, NS, ZE},
    {ZE, NS, NS, NS, NS, NS, ZE},
    {ZE, ZE, ZE, ZE, ZE, ZE, ZE},
    {PB, NS, PS, PS, PS, PS, PB},
    {PB, PM, PM, PM, PS, PS, PB}};

void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule, float (*fuzzyRuleKp)[7], float (*fuzzyRuleKi)[7], float (*fuzzyRuleKd)[7],
                     float kpRatio, float kiRatio, float kdRatio,
                     float eStep, float ecStep)
{
    if (fuzzyRuleKp == NULL) fuzzyRule->FuzzyRuleKp = FuzzyRuleKpRAW;
    else fuzzyRule->FuzzyRuleKp = fuzzyRuleKp;

    if (fuzzyRuleKi == NULL) fuzzyRule->FuzzyRuleKi = FuzzyRuleKiRAW;
    else fuzzyRule->FuzzyRuleKi = fuzzyRuleKi;

    if (fuzzyRuleKd == NULL) fuzzyRule->FuzzyRuleKd = FuzzyRuleKdRAW;
    else fuzzyRule->FuzzyRuleKd = fuzzyRuleKd;

    fuzzyRule->KpRatio = kpRatio;
    fuzzyRule->KiRatio = kiRatio;
    fuzzyRule->KdRatio = kdRatio;

    if (eStep < 0.00001f) eStep = 1;
    if (ecStep < 0.00001f) ecStep = 1;
    fuzzyRule->eStep = eStep;
    fuzzyRule->ecStep = ecStep;
}

void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref)
{
    float eLeftTemp, ecLeftTemp;
    float eRightTemp, ecRightTemp;
    int eLeftIndex, ecLeftIndex;
    int eRightIndex, ecRightIndex;

    fuzzyRule->dt = 1;

    fuzzyRule->e = ref - measure;
    fuzzyRule->ec = (fuzzyRule->e - fuzzyRule->eLast) / fuzzyRule->dt;
    fuzzyRule->eLast = fuzzyRule->e;

    // 隶属区间计算
    eLeftIndex = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 6 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e >= 0 ? ((int)(fuzzyRule->e / fuzzyRule->eStep) + 3) : ((int)(fuzzyRule->e / fuzzyRule->eStep) + 2)));
    eRightIndex = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 6 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e >= 0 ? ((int)(fuzzyRule->e / fuzzyRule->eStep) + 4) : ((int)(fuzzyRule->e / fuzzyRule->eStep) + 3)));
    ecLeftIndex = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 6 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec >= 0 ? ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 3) : ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 2)));
    ecRightIndex = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 6 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec >= 0 ? ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 4) : ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 3)));

    // 隶属度计算
    eLeftTemp = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 1 : (eRightIndex - fuzzyRule->e / fuzzyRule->eStep - 3));
    eRightTemp = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 1 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e / fuzzyRule->eStep - eLeftIndex + 3));
    ecLeftTemp = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 1 : (ecRightIndex - fuzzyRule->ec / fuzzyRule->ecStep - 3));
    ecRightTemp = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 1 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec / fuzzyRule->ecStep - ecLeftIndex + 3));

    fuzzyRule->KpFuzzy = eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKp[eLeftIndex][ecLeftIndex] +
                         eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKp[eLeftIndex][ecRightIndex] +
                         eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKp[eRightIndex][ecLeftIndex] +
                         eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKp[eRightIndex][ecRightIndex];

    fuzzyRule->KiFuzzy = eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKi[eLeftIndex][ecLeftIndex] +
                         eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKi[eLeftIndex][ecRightIndex] +
                         eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKi[eRightIndex][ecLeftIndex] +
                         eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKi[eRightIndex][ecRightIndex];

    fuzzyRule->KdFuzzy = eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKd[eLeftIndex][ecLeftIndex] +
                         eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKd[eLeftIndex][ecRightIndex] +
                         eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKd[eRightIndex][ecLeftIndex] +
                         eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKd[eRightIndex][ecRightIndex];
}

/******************************* PID CONTROL *********************************/
// PID优化环节函数声明
static void f_Trapezoid_Intergral(PID_t *pid);
static void f_Integral_Limit(PID_t *pid);
static void f_Derivative_On_Measurement(PID_t *pid);
static void f_Changing_Integration_Rate(PID_t *pid);
static void f_Output_Filter(PID_t *pid);
static void f_Derivative_Filter(PID_t *pid);
static void f_Output_Limit(PID_t *pid);
static void f_Proportion_Limit(PID_t *pid);
static void f_PID_ErrorHandle(PID_t *pid);

/**
 * @brief          PID初始化   PID initialize
 * @param[in]      PID结构体   PID structure
 * @param[in]      略
 * @retval         返回空      null
 */
void PID_Init(
    PID_t *pid,
    float max_out,
    float intergral_limit,

    float kpid[3],

    float A,
    float B,

    float output_lpf_rc,
    float derivative_lpf_rc,

    uint16_t ols_order,

    uint8_t improve)
{
    pid->IntegralLimit = intergral_limit;//积分限幅
    pid->MaxOut = max_out;//总输出限幅
    pid->Ref = 0;

    pid->Kp = kpid[0];
    pid->Ki = kpid[1];
    pid->Kd = kpid[2];
    pid->ITerm = 0;

    // 变速积分参数
    // coefficient of changing integration rate
    pid->CoefA = A;
    pid->CoefB = B;

    pid->Output_LPF_RC = output_lpf_rc;//总输出低通滤波

    pid->Derivative_LPF_RC = derivative_lpf_rc;//微分低通滤波

    // DWT定时器计数变量清零
    // reset DWT Timer count counter
    pid->DWT_CNT = 0;

    // 设置PID优化环节
    pid->Improve = improve;

    // 设置PID异常处理 目前仅包含电机堵转保护
    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->Output = 0;
}

void PID_set(PID_t *pid, float kpid[3])
{
    pid->Kp = kpid[0];
    pid->Ki = kpid[1];
    pid->Kd = kpid[2];
}

/**
 * @brief          PID计算
 * @param[*pid]      PID结构体
 * @param[measure]   测量值
 * @param[ref]       期望值
 * @retval         返回空
 */
float PID_Calculate(PID_t *pid, float measure, float ref)
{
    if (pid->Improve & ErrorHandle)
        f_PID_ErrorHandle(pid);

    /*uint32_t tmp = pid->DWT_CNT;
    pid->dt = DWT_GetDeltaT(&tmp);*/
    pid->dt = 1;  // 差分形式

    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    if (pid->User_Func1_f != NULL)
        pid->User_Func1_f(pid);

    if (pid->FuzzyRule == NULL)
    {
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
    }

    if (pid->User_Func2_f != NULL)
        pid->User_Func2_f(pid);

    // 梯形积分
    if (pid->Improve & Trapezoid_Intergral)
        f_Trapezoid_Intergral(pid);
    // 变速积分
    if (pid->Improve & ChangingIntegrationRate)
        f_Changing_Integration_Rate(pid);
    // 微分先行
    if (pid->Improve & Derivative_On_Measurement)
        f_Derivative_On_Measurement(pid);
    // 微分滤波器
    if (pid->Improve & DerivativeFilter)
        f_Derivative_Filter(pid);
    // 积分限幅
    if (pid->Improve & Integral_Limit)
        f_Integral_Limit(pid);

    pid->Iout += pid->ITerm;

    pid->Output = pid->Pout + pid->Iout + pid->Dout;

    // 输出滤波
    if (pid->Improve & OutputFilter)
        f_Output_Filter(pid);

    // 输出限幅
    f_Output_Limit(pid);

    // 无关紧要
    f_Proportion_Limit(pid);

    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;
    return pid->Output;
}

/**
 * @brief          清空PID历史状态与输出
 * @param[*pid]    PID结构体指针
 * @retval         返回清零后的输出 (0.0f)
 */
float PID_Clear(PID_t *pid)
{
    if (pid == NULL) {
        return 0.0f;
    }

    // 1. 清空当前计算量
    pid->Measure = 0.0f;
    pid->Ref     = 0.0f;
    pid->Err     = 0.0f;

    // 2. 清空输出与各项中间结果
    pid->Pout    = 0.0f;
    pid->ITerm   = 0.0f;
    pid->Dout    = 0.0f;
    pid->Iout    = 0.0f;    // 【核心】清空累加积分，防止切回正常模式时出现积分饱和冲刺
    pid->Output  = 0.0f;

    // 3. 清空历史状态记录
    pid->Last_Measure = 0.0f; // 【核心】防止微分先行 (Derivative_On_Measurement) 突变
    pid->Last_Output  = 0.0f; // 防止输出滤波异常
    pid->Last_Dout    = 0.0f; // 防止微分滤波异常
    pid->Last_Err     = 0.0f; // 【核心】防止标准微分项计算 (Err - Last_Err) 产生巨大尖峰
    pid->Last_ITerm   = 0.0f; // 防止梯形积分异常

    return pid->Output;
}
static void f_Trapezoid_Intergral(PID_t *pid)
{
    if (pid->FuzzyRule == NULL)
        pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
    else
        pid->ITerm = (pid->Ki + pid->FuzzyRule->KiFuzzy) * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

static void f_Changing_Integration_Rate(PID_t *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        // Integral still increasing
        if (abs(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PID_t *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            // 积分呈累积趋势
            // Integral still increasing
            pid->ITerm = 0;
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(PID_t *pid)
{
    if (pid->FuzzyRule == NULL)
    {
        pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
    }
    else
    {
        pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * (pid->Last_Measure - pid->Measure) / pid->dt;
    }
}

static void f_Derivative_Filter(PID_t *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

static void f_Output_Filter(PID_t *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

static void f_Output_Limit(PID_t *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

static void f_Proportion_Limit(PID_t *pid)
{
    if (pid->Pout > pid->MaxOut)
    {
        pid->Pout = pid->MaxOut;
    }
    if (pid->Pout < -(pid->MaxOut))
    {
        pid->Pout = -(pid->MaxOut);
    }
}

// PID ERRORHandle Function
static void f_PID_ErrorHandle(PID_t *pid)
{
    /*Motor Blocked Handle*/
    if (pid->Output < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}

/*************************** FEEDFORWARD CONTROL *****************************/
/**
 * @brief          前馈控制初始化
 * @param[in]      前馈控制结构体
 * @param[in]      略
 * @retval         返回空
 */
void Feedforward_Init(
    Feedforward_t *ffc,
    float max_out,
    float *c,
    float lpf_rc,
    uint16_t ref_dot_ols_order,
    uint16_t ref_ddot_ols_order)
{
    ffc->MaxOut = max_out;

    // 设置前馈控制器参数 详见前馈控制结构体定义
    // set parameters of feed-forward controller (see struct definition)
    if (c != NULL && ffc != NULL)
    {
        ffc->c[0] = c[0];
        ffc->c[1] = c[1];
        ffc->c[2] = c[2];
    }
    else
    {
        ffc->c[0] = 0;
        ffc->c[1] = 0;
        ffc->c[2] = 0;
        ffc->MaxOut = 0;
    }

    //低通滤波参数
    ffc->LPF_RC = lpf_rc;

    // 最小二乘提取信号微分初始化
    // differential signal is distilled by OLS
    ffc->Ref_dot_OLS_Order = ref_dot_ols_order;
    ffc->Ref_ddot_OLS_Order = ref_ddot_ols_order;
    if (ref_dot_ols_order > 2)
        OLS_Init(&ffc->Ref_dot_OLS, ref_dot_ols_order);
    if (ref_ddot_ols_order > 2)
        OLS_Init(&ffc->Ref_ddot_OLS, ref_ddot_ols_order);

    ffc->DWT_CNT = DWT->CYCCNT;

    ffc->Output = 0;
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float Feedforward_Calculate(Feedforward_t *ffc, float ref)
{
	//求离散后的单位时间
    ffc->dt = 1; // 离散控制：单位时间固定为 1（差分形式）
	//将期望值进行一阶低通滤波
    ffc->Ref = ref * ffc->dt / (ffc->LPF_RC + ffc->dt) +
               ffc->Ref * ffc->LPF_RC / (ffc->LPF_RC + ffc->dt);
    /*公式解析
    ffc->Ref = ref * ffc->dt / (ffc->LPF_RC + ffc->dt) + ffc->Ref * ffc->LPF_RC / (ffc->LPF_RC + ffc->dt);
             = ref * (1/(LPF_RC/ffc->dt + 1)) + ffc->Ref * (1/(ffc->dt/LPF_RC + 1))
             = ref * A + ffc->Ref * (1-A)
    A   = 1/(LPF_RC/ffc->dt + 1)
    1-A = 1/(ffc->dt/LPF_RC + 1)
    注：https://blog.csdn.net/qq_37662088/article/details/125075600
    */

    // 计算一阶导数
    // calculate first derivative
    if (ffc->Ref_dot_OLS_Order > 2)
        ffc->Ref_dot = OLS_Derivative(&ffc->Ref_dot_OLS, ffc->dt, ffc->Ref);
    else
        ffc->Ref_dot = (ffc->Ref - ffc->Last_Ref) / ffc->dt;
    // 计算二阶导数
    // calculate second derivative
    if (ffc->Ref_ddot_OLS_Order > 2)
        ffc->Ref_ddot = OLS_Derivative(&ffc->Ref_ddot_OLS, ffc->dt, ffc->Ref_dot);
    else
        ffc->Ref_ddot = (ffc->Ref_dot - ffc->Last_Ref_dot) / ffc->dt;
    // 计算前馈控制输出
    // calculate feed-forward controller output
    ffc->Output = ffc->c[0] * ffc->Ref + ffc->c[1] * ffc->Ref_dot + ffc->c[2] * ffc->Ref_ddot;

    ffc->Output = MATH_Limit_float(ffc->Output, -ffc->MaxOut, ffc->MaxOut);

    ffc->Last_Ref = ffc->Ref;

    ffc->Last_Ref_dot = ffc->Ref_dot;

    return ffc->Output;
}


/*************************** Tracking Differentiator ***************************/


void TD_Init(TD_t *td, float r, float h0)
{
    td->r = r;    //微分器的比例系数。它用于调整微分器的响应强度
    td->h0 = h0;  //表示微分器的初始值。它是微分器的初始状态

    td->x = 0;
    td->dx = 0;   //表示微分器的状态变量，即微分器的当前状态。它表示微分器的输出
    td->ddx = 0;  //表示微分器的状态变量的微分，即微分器的当前变化率
    td->last_dx = 0;
    td->last_ddx = 0;
    td->DWT_CNT = DWT->CYCCNT;
}




/*入口参数  TD_t的类型的指针   浮点数  input   返回值  返回
1.在dwt_cnt的地址下取出时间间隔   td->dt
*/
float TD_Calculate(TD_t *td, float input)
{
    static float d, a0, y, a1, a2, a, fhan;

    td->dt = 1; // 离散控制：单位时间固定为 1（差分形式）

    td->Input = input;

    d = td->r * td->h0 * td->h0;//它用于表示函数的变化率或斜率
    a0 = td->dx * td->h0;       //输入信号的初始值或初值
    y = td->x - td->Input + a0; //微分器的输出信号，表示对输入信号进行微分后得到的结果
    a1 = Sqrt(d * (d + 8 * abs(y)));//它根据斜率与输出乘法、加法和开平方运算得到                   系数1
    a2 = a0 + sign(y) * (a1 - d) / 2;  //它根据初始值a0、输出y和系数a1的值计算得到                系数2
    a = (a0 + y) * (sign(y + d) - sign(y - d)) / 2 + a2 * (1 - (sign(y + d) - sign(y - d)) / 2);//系数3
    fhan = -td->r * a / d * (sign(a + d) - sign(a - d)) / 2 -
           td->r * sign(a) * (1 - (sign(a + d) - sign(a - d)) / 2);//

    td->ddx = fhan;
    td->dx += (td->ddx + td->last_ddx) * td->dt / 2;
    td->x += (td->dx + td->last_dx) * td->dt / 2;

    td->last_ddx = td->ddx;
    td->last_dx = td->dx;

    return td->x;
}