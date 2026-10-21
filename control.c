#include "control.h"
#include "menu.h"
#include "image.h"
#include "wifi.h"

//宏定义
#define MIT_A (ATOM1_CH1_P33_9)
#define MIT_B (ATOM1_CH2_P33_11)
#define MIT_C (ATOM1_CH5_P32_4)
#define MIT_D (ATOM2_CH4_P33_12)
#define DUOJI (ATOM3_CH1_P33_5)
#define UP_LIMIT (100)
#define DOWN_LIMIT (-100)
#define DUOJI_MID (800)
#define ENC_L_TIM       (TIM2_ENCODER)              //计数器
#define ENC_L_A_PIN     (TIM2_ENCODER_CH1_P33_7)    //脉冲脚
#define ENC_L_DIR_PIN   (TIM2_ENCODER_CH2_P33_6)    //方向脚
#define ENC_R_TIM       (TIM6_ENCODER)
#define ENC_R_A_PIN     (TIM6_ENCODER_CH1_P20_3)
#define ENC_R_DIR_PIN   (TIM6_ENCODER_CH2_P20_0)
#define SPEED_SCALE     (100.0f)//换算系数
#define DUTY_MAX_RUN    (2000)//防止疯转
#define DUTY_MIN_RUN    (300)//防止太慢
#define DIFF_K (0.2)//转向系数
volatile uint8 g_lost=1;//上电先当丢线，防止没图就发车
volatile uint8 g_frame_new=0;//输出新的一帧图像
volatile uint8 g_image_ready=0;//得到一帧新的图像
volatile int16 out;
//打包结构体，写到函数省事
typedef struct
{
    float Target;//目标
    float Current;//现在
    float Out;//输出
    float Bias_0;//本次偏差
    float Bias_1;//上次偏差
    float Bias_2;//上上次偏差
    float KP;
    float KI;
    float KD;
} pid_parameter;

pid_parameter bh_pid_left = {0};//存参数，先初始化
pid_parameter bh_pid_right = {0};
//初始化
void control_init(void)
{
    pwm_init(MIT_A, 20000, 0);//第一个值是频率，第二个值是初始占空比
    pwm_init(MIT_B, 20000, 0);
    pwm_init(MIT_C, 20000, 0);
    pwm_init(MIT_D, 20000, 0);
    pwm_init(DUOJI, 50,DUOJI_MID);
    encoder_dir_init(ENC_L_TIM, ENC_L_A_PIN, ENC_L_DIR_PIN);
    encoder_dir_init(ENC_R_TIM, ENC_R_A_PIN, ENC_R_DIR_PIN);
    encoder_clear_count(ENC_L_TIM);
    encoder_clear_count(ENC_R_TIM);
//定时器中断初始化，给电机用
pit_init(CCU60_CH0,5000);
}
//舵机函数_丢线保护_限幅保护
void ImageMain(void)
{   static int last_err=0;
    static uint8 ever_found=0;  //是否曾经成功找到过线
    g_frame_new=1;
    g_image_ready=1;
    int err;
    int err_sum=0;
    int err_each;
    int cha;//偏差值
    int n=0;

    int r;
    for(r=35;r<=60;r++)//丢线保护和多行平均
    {
        if(mid_line[r]!=255)
        {
            n++;
            err_each=mid_line[r]-94;
            err_sum=err_sum+err_each;
        }


    }
    if(n <3)
    {
        g_lost=1;
        return;
    }

    if(ever_found==0)
    {
        ever_found=1;
        g_lost=0;//先让车跑，只要是n<4了，g_lost的值就永远不会是0了，也就是只要丢线，车就永远不跑了
    }
    err=err_sum/n;
    cha=err-last_err;
    last_err=err;
    int dj_out=(menu_get_param(0)*err+menu_get_param(2)*cha)/1000;
    if(dj_out>=UP_LIMIT)//限幅保护
    {
        dj_out=UP_LIMIT;
    }
    if(dj_out<DOWN_LIMIT)
    {
        dj_out=DOWN_LIMIT;
    }

    pwm_set_duty(DUOJI,DUOJI_MID-dj_out);//舵机输出公式   out>0,舵机往右偏，out<0，舵机往左偏
    out=(int16)dj_out;

}
//存值函数_清零函数_浮点数向整数转换函数（输出速度）
void getvalue(pid_parameter *pid)
{
    pid->Bias_2=pid->Bias_1;
    pid->Bias_1=pid->Bias_0;
    pid->Bias_0=pid->Target-pid->Current;
    pid->Out+=pid->KP*(pid->Bias_0-pid->Bias_1)
            +pid->KI*(pid->Bias_0)
            +pid->KD*(pid->Bias_0-2.0f*pid->Bias_1+pid->Bias_2);
}
static void deletevalue(pid_parameter *pid)
{
    pid->Bias_0=0;
    pid->Bias_1=0;
    pid->Bias_2=0;
    pid->Out=0;
}
static uint32 transout(pid_parameter *pid)
{
    if(pid->Out > (float)DUTY_MAX_RUN)
    {
        pid->Out = (float)DUTY_MAX_RUN;
    }
    else if(pid->Out < 0.0f)
    {
        pid->Out = 0.0f;
        return 0;
    }
    else if(pid->Out < (float)DUTY_MIN_RUN)
    {
        return DUTY_MIN_RUN;                        // 死区只把输出垫上来，不回写 Out
    }

    return (uint32)pid->Out;
}

//电机转动_丢线保护
void CtrlMain(void)
{
    float x=DIFF_K*out/100.0f;
    if(x>=0.5f)
    {
        x=0.5f;
    }
    if(x<=-0.5f)
    {
        x=-0.5f;
    }
    float base_speed=(float)menu_get_param(3)/SPEED_SCALE;
    int16 bh_speed_left= encoder_get_count(ENC_L_TIM);//读计数器的值
    int16 bh_speed_right= encoder_get_count(ENC_R_TIM);
    encoder_clear_count(ENC_L_TIM);//读完清零
    encoder_clear_count(ENC_R_TIM);
    bh_pid_left.Current=bh_speed_left;//目前速度
    bh_pid_right.Current=bh_speed_right;
    bh_pid_left.Target=(1+x)*base_speed;
    bh_pid_left.KP=(float)menu_get_param(4)/1000.0f;
    bh_pid_left.KI=(float)menu_get_param(1)/1000.0f;
    bh_pid_left.KD=0.0f;//增量式pi，不用d
    bh_pid_right.Target=(1-x)*base_speed;
    bh_pid_right.KP=bh_pid_left.KP;
    bh_pid_right.KI=bh_pid_left.KI;
    bh_pid_right.KD=0.0f;
    static uint32 diuxian_dushu=0;
    if(menu_run_flag==1)
    {
        diuxian_dushu=diuxian_dushu+1;
    }
    else
    {
        diuxian_dushu=0;
    }


    if(menu_run_flag==1 && diuxian_dushu>=200 && g_lost==0)
    {

        getvalue(&bh_pid_left);//输入参数
        getvalue(&bh_pid_right);

        dianjizhuandong(transout(&bh_pid_left),transout(&bh_pid_right));//输入速度
    }
    else
    {
        dianjizhuandong(0,0);
        deletevalue(&bh_pid_left);//只要停了，马上清零
        deletevalue(&bh_pid_right);

    }


    {
        float wave[3];
        wave[0] = bh_pid_left.Target;
        wave[1] = bh_pid_left.Current;     // 外轮实际速度
        wave[2] = bh_pid_right.Current;    // 内轮实际速度
        SendDataToScope(WIFI_UART, wave, 3);
    }

}
void dianjizhuandong (uint32 left,uint32 right)
{
    pwm_set_duty(MIT_A,left);
    pwm_set_duty(MIT_B,0);
    pwm_set_duty(MIT_C,right);
    pwm_set_duty(MIT_D,0);
}












































