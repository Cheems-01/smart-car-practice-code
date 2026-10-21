#include "control.h"
#include "menu.h"
#include "image.h"

//宏定义
#define MIT_A (ATOM1_CH1_P33_9)
#define MIT_B (ATOM1_CH2_P33_11)
#define MIT_C (ATOM1_CH5_P32_4)
#define MIT_D (ATOM2_CH4_P33_12)
#define DUOJI (ATOM3_CH1_P33_5)
#define UP_LIMIT (100)
#define DOWN_LIMIT (-100)
#define DUOJI_MID (800)

volatile uint8 g_lost=1;//上电先当丢线，防止没图就发车
volatile uint8 g_frame_new=0;//输出新的一帧图像
volatile uint8 g_image_ready=0;//得到一帧新的图像
//初始化
void control_init(void)
{
    pwm_init(MIT_A, 20000, 0);//第一个值是频率，第二个值是初始占空比
    pwm_init(MIT_B, 20000, 0);
    pwm_init(MIT_C, 20000, 0);
    pwm_init(MIT_D, 20000, 0);
    pwm_init(DUOJI, 50,DUOJI_MID);
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
    int out=(menu_get_param(0)*err+menu_get_param(2)*cha)/1000;
    if(out>=UP_LIMIT)//限幅保护
    {
        out=UP_LIMIT;
    }
    if(out<DOWN_LIMIT)
    {
        out=DOWN_LIMIT;
    }
    pwm_set_duty(DUOJI,DUOJI_MID-out);//舵机输出公式
}
//电机转动_丢线保护
void CtrlMain(void)
{
    static uint32 dushu=0;
    if(menu_run_flag==1)
    {
        dushu=dushu+1;
    }
    else
    {
        dushu=0;
    }

    if(menu_run_flag==1 && dushu>=200 && g_lost==0)
    {
        dianjizhuandong(menu_get_param(3));
    }
    else
    {
        dianjizhuandong(0);
    }

}
void dianjizhuandong (uint32 duty)
{
    pwm_set_duty(MIT_A,duty);
    pwm_set_duty(MIT_B,0);
    pwm_set_duty(MIT_C,duty);
    pwm_set_duty(MIT_D,0);
}












































