#include "app_mpu6050.h"
#include "i2c.h"//导入硬件i2c的头文件
#include "delay.h"
#include "si2c.h"
#include "task.h"
#include "math.h"
#include "qmath.h"
static void reg_write(uint8_t reg, uint8_t value);
static uint8_t reg_read(uint8_t reg);

static float ax,ay,az;//加速度计的结果，单位g
static float gx,gy,gz;//陀螺仪测量结果，单位度/s
static float temperature;//温度计的结果，单位摄氏度

static float yaw,pitch,roll,yaw_g,pitch_g,roll_g,pitch_a,roll_a;//偏航角、俯仰角、翻滚角  单位为度


SI2C_TypeDef SI2C1 =
{
    GPIOA,
    GPIO_Pin_5,//SCL

    GPIOA,
    GPIO_Pin_6//SDA
};
//使用软件I2C代替硬件I2C，不知道为什么我这个硬件I2C无法使用

//
//@简介：MPU6050初始化
//
void App_MPU6050_Init(void)
{
//	//1.初始化I2C总线   SCL  PB8  SDA  PB9
//	//需要进行重映射 属于AFIO模块
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
//	GPIO_PinRemapConfig(GPIO_Remap_I2C1 , ENABLE);
//	
//	//2.将GPIO引脚都设置为复用开漏模式
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//	GPIO_InitTypeDef GPIO_InitStructure = {0};
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
//	GPIO_Init(GPIOB,&GPIO_InitStructure);
//	
//	//3.对I2C进行配置
//	//开启I2C1的时钟
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
//	
//	I2C_InitTypeDef I2C_InitSturcture = {0};
//	I2C_InitSturcture.I2C_Mode = I2C_Mode_I2C;
//	I2C_InitSturcture.I2C_ClockSpeed = 400000;//I2C的通信速度400kHz
//	I2C_InitSturcture.I2C_DutyCycle = I2C_DutyCycle_2;//时钟SCL选择2：1的占空比
//	I2C_InitSturcture.I2C_OwnAddress1 = 0x00;
//	I2C_InitSturcture.I2C_Ack = I2C_Ack_Enable;
//	I2C_InitSturcture.I2C_AcknowledgedAddress= I2C_AcknowledgedAddress_7bit;
//	I2C_Init(I2C1,&I2C_InitSturcture);
//	
//	I2C_Cmd(I2C1,ENABLE);
	My_SI2C_Init(&SI2C1);
	//4.使用I2C接口配置MPU6050的寄存器
	//查阅手册，首先向寄存器107（10进制，控制MPU6050的休眠状态） 16进制编号0x6b 里的8个寄存器写入10000000(16进制0x80)，关闭休眠模式
	reg_write(0x6b, 0x80);//复位
	
	//这里是否可以使用阻塞式延时？？？答案是可以，初始化可以，但进程函数Proc中不能使用阻塞式延时
	Delay(100);//复位时间较长，延长时间
	
	//再写入0x00将MPU6050从休眠模式唤醒
	reg_write(0x6b, 0x00);

	//查阅手册，找到控制陀螺仪量程的寄存器27（10进制）  16进制编号0x1b 里的8个寄存器写入00011000（16进制0x18） 
	//控制量程的寄存器FS_SEL 位于 Bit4 和 Bit3  满量程只需将Bit4和Bit3置1
	reg_write(0x1b,0x18);
	//查阅手册，找到控制加速计量程的寄存器28（10进制）  16进制编号0x1c 里的8个寄存器写入00000000（16进制0x00） 
	//控制量程的寄存器AFS_SEL 位于 Bit4 和 Bit3  设置为+2g量程只需将Bit4和bit3置0
	reg_write(0x1c,0x00);
	
	
}

//
//@简介：MPU6050的进程函数
//进程函数不允许使用延时函数，因为要保证单片机多个任务并行运行
//
void App_MPU6050_Proc(void)
{
//	static uint32_t nxt = 0;//nxt: next run time下次程序运行的时间
//	if(GetTick() <nxt ) return ;	
//	nxt+=5;//nxt = nxt +5;传感器5ms延迟更新一次，需要延时5ms
	
	//使用PERIODIC(T)代替上述三行代码,无需加分号，就表示进程函数每5ms延时一次
	PERIODIC(5) //这个5ms也是互补滤波器的周期，可以得到采样频率为1/T为200Hz
	App_MPU6050_Update();//更新传感器的值,这样就可以使用陀螺仪和加速计测得的数据了

//	//通过陀螺仪的测量结果计算欧拉角 Δt取0.005秒
	yaw_g   = yaw + gz * 0.005;
	pitch_g = pitch + gx * 0.005;
	roll_g  = roll - gy * 0.005;
	//通过加速度计解算欧拉角 
//	float yaw_a   = 0;//俯仰角无法使用加速度计解算
	pitch_a = qatan2(ay,az)/3.1415927f  * 180;//将弧度制转换为角度制
	roll_a  = qatan2(ax,az)/3.1415927f  * 180;
	
	
	//使用互补滤波器对陀螺仪和加速度计的结果进行融合
	float alpha = 0.95238f;
	
	yaw    = alpha * yaw_g;
	pitch  = alpha * pitch_g + (1.0f-alpha) * pitch_a;
	roll   = alpha * roll_g  + (1.0f-alpha) * roll_a;
//	Delay(5);//传感器5ms延迟更新一次，需要延时5ms
}   

//
//@简介：获取偏航角YAW的值
//
float App_MPU6050_GetYaw(void)
{
	return yaw;
}

//
//@简介：获取俯仰角PITCH的值
//
float App_MPU6050_GetPitch(void)
{
	return pitch;
}

//
//@简介：获取翻滚角ROLL的值
//
float App_MPU6050_GetRoll(void)
{
	return roll;
}









//
//@简介：更新MPU6050的值，得到MPU6050获取的数据
//
void App_MPU6050_Update(void)
{
	//uint8_t accel_xout_h = reg_read(0x3b);
	//uint8_t accel_xout_l = reg_read(0x3c);
	//int16_t ax_raw = (accel_xout_h<<8)+accel_xout_l;
	//加速度计获取的x轴加速度数据（有向-->有符号） 高位字节和低位字节分别保存在寄存器0x3b（高）和0x3c（低），需要将它们拼接起来
	int16_t ax_raw = (int16_t)(reg_read(0x3b)<<8)+reg_read(0x3c);
	//同理，依次读取y轴ay，z轴的结果az
	int16_t ay_raw = (int16_t)(reg_read(0x3d)<<8)+reg_read(0x3e);//ay
	int16_t az_raw = (int16_t)(reg_read(0x3f)<<8)+reg_read(0x40);//az
	
	ax = ax_raw * 6.1035e-5f;
	ay = ay_raw * 6.1035e-5f;
	az = az_raw * 6.1035e-5f;
	
	//读取温度计的数据
	int16_t temperature_raw = (int16_t)(reg_read(0x41)<<8)+reg_read(0x42);//温度计的数据
	temperature =  temperature_raw /340 +36.53;//针对MPU6050
	
	//读取陀螺仪获取的数据
	int16_t gx_raw = (int16_t)(reg_read(0x43)<<8)+reg_read(0x44);//gx
	int16_t gy_raw = (int16_t)(reg_read(0x45)<<8)+reg_read(0x46);//gy
	int16_t gz_raw = (int16_t)(reg_read(0x47)<<8)+reg_read(0x48);//gz
	
	gx = gx_raw * 6.1035e-2f;
	gy = gy_raw * 6.1035e-2f;
	gz = gz_raw * 6.1035e-2f;
}

//
//@简介：获取x轴向加速度，单位g
//
float App_MPU6050_GetAx(void)
{
	return ax;
}

//
//@简介：获取y轴向加速度，单位g
//
float App_MPU6050_GetAy(void)
{
	return ay;
}

//
//@简介：获取z轴向加速度，单位g
//
float App_MPU6050_GetAz(void)
{
	return az;
}

//
//@简介：绕x轴向的角速度，单位度/s
//
float App_MPU6050_GetGx(void)
{
	return gx;
}

//
//@简介：绕y轴向的角速度，单位度/s
//
float App_MPU6050_GetGy(void)
{
	return gy;
}
	
//
//@简介：绕z轴向的角速度，单位度/s
//
float App_MPU6050_GetGz(void)
{
	return gz;
}
	
//
//@简介：获取温度计的值，单位摄氏度
//
float App_MPU6050_GetTemperature(void)
{
	return temperature;
}

//
//@简介：向寄存器写值
//@步骤：同时发送寄存器的地址和值
//@参数：reg:要写入的寄存器的导致  value：要写入的值
static void reg_write(uint8_t reg, uint8_t value)
{
	uint8_t byteToSend[] = {reg , value};//定义要发送的数组
	My_SI2C_SendBytes(&SI2C1, 0xd0 ,byteToSend , 2 );
}

//
//@简介：读取寄存器的值
//@步骤：①发送寄存器的地址②读取寄存器的值
//@参数：reg：要读取寄存器的地址
//@返回值：表示读取到的值
static uint8_t reg_read(uint8_t reg)
{
	My_SI2C_SendBytes(&SI2C1, 0xd0, &reg,1);//发送寄存器的地址
	
	uint8_t regValue;//提前准备一个缓冲区
	My_SI2C_ReceiveBytes(&SI2C1,0xd0,&regValue,1);//使用I2C读取数据，事先定义一个buffer缓冲区进行存储
	
	return regValue;
}

