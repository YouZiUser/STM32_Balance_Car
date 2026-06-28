#include "app_control.h"
#include "app_pid.h"
#include "app_mpu6050.h"
#include "task.h"
#include "qmath.h"
#include "app_motor.h"
#include "app_encoder.h"
//为角速度环和角度环声明两个PID变量
//按照环由外到内的顺序声明PID变量
static PID_TypeDef pid_velocity;//声明速度环的PID控制器  velocity速度
static PID_TypeDef pid_theta;//theta角度环的PID控制器
static PID_TypeDef pid_theta_dot;//角速度环的PID控制器
static PID_TypeDef pid_turn;//转速系统即转向环的PID控制器

static const float g = 9.8f;//将重力加速度作为一个变量声明出来
//const关键字是表示g是一个不能被修改的变量，也就是常量，不会被改变，存储在rom/flash（单片机的rom）里面  
//而如果不带const，则默认g是一个变量，存储在ram内存里面
static const float lp = 0.062f;//平衡车中心到电池中心的距离，即是平衡车倒立摆的长度
static const float rw = 0.032f;//平衡车轮胎半径
//
//@简介：负责初始化平衡车倒立摆的控制系统
//
void App_Control_Init(void)
{
	//对速度环的PID进行初始化
	PID_Init(&pid_velocity,10.0f,1.0f,0.0f);
	//对速度环PID的控制器输出x..ref（平衡车的线加速度）限幅在+-0.5g之间
	PID_LimitConfig(&pid_velocity,+0.5f*g,-0.5f*g);
	
	//对角度环PID进行初始化
	PID_Init(&pid_theta, 4.0f, 0.0f, 0.0f);//对角度环进行初始化
	//需要对角度环PID的控制器输出θ.ref进行限幅在-4pi - +4pi之间
	PID_LimitConfig(&pid_theta, +4.0f * 3.1415926f,-4.0f * 3.1415926f );
	
	//对角速度环PID进行初始化
	PID_Init(&pid_theta_dot,10.0f, 10.0f, 0.0f);//对角速度环进行初始化	
	//需要对角速度环PID的控制器输出θ..ref进行限幅在-40pi - +40pi之间
	PID_LimitConfig(&pid_theta_dot,+40.0f * 3.1415926f,-40.0f * 3.1415926f);

	//对转向环PID进行初始化
	PID_Init(&pid_turn,1.0f,0.0f,0.0f);//转向环只有一个比例环节，且Kp=1
	//需要对转向环PID的控制器输出转向差ωdiff进行限幅在-15rad/s - +15rad/s之间
	PID_LimitConfig(&pid_turn, 15.0f,-15.0f);
}


static float omega_ref =0.0f;//防止进程函数执行一次后，这个变量的值需要清零
static uint64_t lastTime =0;
//
//@简介：负责平衡车倒立摆的进程函数，用来执行PID（串级PID的实现）
//需要在main函数的while循环中调用
void App_Control_Proc(void)
{
	PERIODIC(5)  //控制程序每5ms执行一次
	
	uint64_t now = GetUs();//读取当前的时间
	float delatT = (now - lastTime) * 1.0e-6;//获取进程函数执行一次的时间，并将单位由μs转换为s
	
	
	//-1、改变速度环的设定值，先设置为0
//	PID_ChangeSP(&pid_velocity, x_dot_def);
	
	//有蓝牙遥控后把上述设定值注释掉即可
	
	//-2、速度环的反馈值FB线速度x_dot.需要由编码器测得的轮胎的角速度值ω去除ω2分量后再经过数学计算得到
	//为保持平衡车稳定，将左右两轮胎的编码器所测得的值进行取平均处理
	float omega = (App_Encoder_GetSpeed_L()+App_Encoder_GetSpeed_R())/2.0f;
	
	//2、最外环角度环的反馈值FB为MPU6050采集到的数据，即俯仰角  由于速度环需要，把这一部分放在此处
	//角速度环的反馈值FB为MPU6050采集的整个平衡车绕x轴的角速度  要注意MPU6050的坐标位置
	float theta     = App_MPU6050_GetPitch()/180.0f  * 3.1415927f;//θ角 并将单位转换为弧度制
	float theta_dot = App_MPU6050_GetGx()/180.0f  * 3.1415927f;//角速度 并将单位转换为弧度制
	
	//-3、计算速度环的反馈值x_dot
	float omega2 = -(theta_dot * (lp+rw)) / rw;//
	float omega1 = omega - omega2;//得到轮胎实际角速度ω1
	float x_dot  = omega1 * rw; //得到轮胎实际转速x_dot，并作为反馈值
	
	//-4、计算速度环的控制器输出x..ref并逆解算x..ref得到θref
	float theta_ref = qatan2(PID_Compute(&pid_velocity,x_dot) , g) ;
	
	
	//1、将角度设定值SPθ设为速度环的输出θref，解决速度环悖论，保持平衡车的平衡
	PID_ChangeSP(&pid_theta, theta_ref);
		
	//3、对最外环的PID进行运算，得到外环PID的控制器输出
	float theta_dot_ref     = PID_Compute(&pid_theta,theta);//命名方式与框图保持一致
	
	//4、将最外环的PID控制器输出theta_dot_ref最为角速度环的设定值
	PID_ChangeSP(&pid_theta_dot,theta_dot_ref);
	
	//5、计算角速度环PID的输出
	float theta_dot_dot_ref = PID_Compute(&pid_theta_dot,theta_dot);
	
	//6、倒立摆数学模型的逆解算
	float x_dot_dot_ref = ( g * qsin(theta) - theta_dot_dot_ref * lp)/qcos(theta);//0.062为平衡车对应倒立摆的长度
	
	//7、由x_dot_dot_ref通过积分器得到轮胎转速的设定值ω_ref
	if(lastTime !=0)//如果lastTime=0，就代表是第一次运行，不进行这个运算，跳过这步，而如果lastTime！=0，则执行这步
	{
		omega_ref +=(1.0f/rw)* x_dot_dot_ref * delatT;
	}
	
	//转向环
	float gz = App_MPU6050_GetGz() * (3.1415926f/ 180.0f);//读出速度环反馈值FB的值gz并转化为弧度每秒
	float omege_diff = PID_Compute(&pid_turn, gz);//输入PID计算得到角速度差omega_diff
	
	float omega_ref_l = omega_ref + omege_diff;//左电机的速度设置为角速度设定值+角速度差
	float omega_ref_r = omega_ref - omege_diff;//右电机的速度设置为角速度设定值-角速度差
	//8、设置轮胎的转速
	App_Motor_SetOmega_L(omega_ref_l);
	App_Motor_SetOmega_R(omega_ref_r);
	
	lastTime = now;//更新lastTime的值
}

//
//@简介：对控制系统进行复位
//
void App_Control_Reset(void)
{
	//1、复位暂存的值
	lastTime = 0.0f;
	omega_ref = 0.0f;
	
	//2、复位PID控制器
	PID_Reset(&pid_velocity);
	PID_Reset(&pid_theta);
	PID_Reset(&pid_theta_dot);
}

//
//@简介：通过遥控器改变平衡车的车速
//@参数：MoveSpeed - 车速 单位m/s
//平衡车的最大行进速度为0.7m/s，根据电机的最大转速及平衡车平衡得到
void App_Control_SetMoveSpeed(float MoveSpeed)
{
	PID_ChangeSP(&pid_velocity, MoveSpeed);
}

//
//@简介：通过遥控器改变平衡车的转向
//@参数：TurnSpeed - 转向速度，单位是rad/s 最大转速为15rad/s
//
void App_Control_SetTurnSpeed(float TurnSpeed)
{
	PID_ChangeSP(&pid_turn, TurnSpeed);
}
