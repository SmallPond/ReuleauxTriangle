  /**
arduino开发环境-灯哥开源FOChttps://gitee.com/ream_d/Deng-s-foc-controller，并安装Kalman。
FOC引脚32, 33, 25, 22    22为enable
AS5600霍尔传感器 SDA-23 SCL-5  MPU6050六轴传感器 SDA-19 SCL-18
本程序有两种平衡方式， FLAG_V为1时使用电压控制，为0时使用速度控制。电压控制时LQR参数使用K1和K2，速度控制时LQR参数使用K3和K4
在wifi上位机窗口中输入：TA+角度，就可以修改平衡角度
比如让平衡角度为90度，则输入：TA90，并且会存入eeprom的位置0中 注：wifi发送命令不能过快，因为每次都会保存进eeprom

在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(5) 中的值，设置为自己的极对数数字，磁铁数量/2。

程序默认设置的供电电压为 12V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值

默认PID针对的电机是 GB2204 ，使用自己的电机需要修改PID参数，才能实现更好效果
 */
#include <SimpleFOC.h>
#include "Command.h"
#include <WiFi.h>
#include <AsyncUDP.h> //引用以使用异步UDP
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "EEPROM.h"
#include "MPU6050.h"

Kalman kalmanZ;
#define gyroZ_OFF -0.19
#define FLAG_V 0
#define DUBUG  1
/* ----IMU Data---- */

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
bool stable = 0;
uint32_t last_unstable_time;

double gyroZangle; // Angle calculate using the gyro only
double compAngleZ; // Calculated angle using a complementary filter
double kalAngleZ;  // 使用卡尔曼滤波计算角度

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
/* ----FOC Data---- */

// driver instance

float constrainAngle(float x);
float controllerLQR(float p_angle, float p_vel, float m_vel);

const char *ssid = "esp32";
const char *password = "12345678";

bool wifi_flag = 0;
AsyncUDP udp;                     //创建UDP对象
unsigned int localUdpPort = 2333; //本地端口号
void wifi_print(char * s,double num);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1);
LowPassFilter lpf_throttle{0.00};

// 倒立摆参数：电压控制使用 K1 K2, 速度控制使用 k3 k4
float LQR_K1_1 = 4;    //摇摆到平衡
float LQR_K1_2 = 1.5;   
float LQR_K1_3 = 0.30; 

float LQR_K2_1 = 3.49;   //平衡态
float LQR_K2_2 = 0.26;   
float LQR_K2_3 = 0.15; 

float LQR_K3_1 = 10;   //摇摆到平衡
float LQR_K3_2 = 1.7;   //
float LQR_K3_3 = 1.75; //

float LQR_K4_1 = 2.4;   //平衡态
float LQR_K4_2 = 1.5;   
float LQR_K4_3 = 1.42; 

//电机参数
BLDCMotor motor = BLDCMotor(5);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);
float target_velocity = 0;
float target_angle = 89.5;
float target_voltage = 0;
float swing_up_voltage = 1.8;
float swing_up_angle = 20;
float v_i_1 = 20;
float v_p_1 = 0.5;
float v_i_2 = 10;
float v_p_2 = 0.2;
//命令设置
Command comm;
bool Motor_enable_flag = 0;
void do_TA(char* cmd) { comm.scalar(&target_angle, cmd);EEPROM.writeFloat(0, target_angle); }
void do_SV(char* cmd) { comm.scalar(&swing_up_voltage, cmd); EEPROM.writeFloat(4, swing_up_voltage); }
void do_SA(char* cmd) { comm.scalar(&swing_up_angle, cmd);EEPROM.writeFloat(8, swing_up_angle); }

void do_START(char* cmd) {  wifi_flag = !wifi_flag; }
void do_MOTOR(char* cmd)
{  
    if(Motor_enable_flag)
        motor.enable();
    else 
        motor.disable();
    Motor_enable_flag = !Motor_enable_flag;
}
#if FLAG_V
void do_K11(char* cmd) { comm.scalar(&LQR_K1_1, cmd); }
void do_K12(char* cmd) { comm.scalar(&LQR_K1_2, cmd); }
void do_K13(char* cmd) { comm.scalar(&LQR_K1_3, cmd); }
void do_K21(char* cmd) { comm.scalar(&LQR_K2_1, cmd); }
void do_K22(char* cmd) { comm.scalar(&LQR_K2_2, cmd); }
void do_K23(char* cmd) { comm.scalar(&LQR_K2_3, cmd); }
#else
void do_vp1(char* cmd) { comm.scalar(&v_p_1, cmd); EEPROM.writeFloat(12, v_p_1);}
void do_vi1(char* cmd) { comm.scalar(&v_i_1, cmd);EEPROM.writeFloat(16, v_i_1); }
void do_vp2(char* cmd) { comm.scalar(&v_p_2, cmd); EEPROM.writeFloat(20, v_p_2);}
void do_vi2(char* cmd) { comm.scalar(&v_i_2, cmd);EEPROM.writeFloat(24, v_i_2); }
void do_tv(char* cmd) { comm.scalar(&target_velocity, cmd); }  // 目标速度
// LRQ 控制参数没有写到 EEPROM 中
void do_K31(char* cmd) { comm.scalar(&LQR_K3_1, cmd); }
void do_K32(char* cmd) { comm.scalar(&LQR_K3_2, cmd); }
void do_K33(char* cmd) { comm.scalar(&LQR_K3_3, cmd); }
void do_K41(char* cmd) { comm.scalar(&LQR_K4_1, cmd); }
void do_K42(char* cmd) { comm.scalar(&LQR_K4_2, cmd); }
void do_K43(char* cmd) { comm.scalar(&LQR_K4_3, cmd); }
#endif


void onPacketCallBack(AsyncUDPPacket packet)
{
    char* da;
    da= (char*)(packet.data());
    Serial.println(da);
    comm.run(da);
    EEPROM.commit();
//  packet.print("reply data");
}
// instantiate the commander
void setup() {
    Serial.begin(115200);
    if (!EEPROM.begin(1000)) {
        Serial.println("Failed to initialise EEPROM");
        Serial.println("Restarting...");
        delay(1000);
        ESP.restart();
    }
    // eeprom 读取
    float nan = EEPROM.readFloat(0);
    if(isnan(nan))
    {
        Serial.println("frist write");
        EEPROM.writeFloat(0, target_angle);     delay(10); EEPROM.commit();
        EEPROM.writeFloat(4, swing_up_voltage); delay(10); EEPROM.commit();
        EEPROM.writeFloat(8, swing_up_angle);   delay(10); EEPROM.commit();
        EEPROM.writeFloat(12, v_p_1);           delay(10); EEPROM.commit();
        EEPROM.writeFloat(16, v_i_1);           delay(10); EEPROM.commit();
        EEPROM.writeFloat(20, v_p_2);           delay(10); EEPROM.commit();
        EEPROM.writeFloat(24, v_i_2);           delay(10); EEPROM.commit();
    }
    else
    {
        target_angle = EEPROM.readFloat(0);
        swing_up_voltage = EEPROM.readFloat(4);
        swing_up_angle = EEPROM.readFloat(8);  
        v_p_1 = EEPROM.readFloat(12);
        v_i_1 = EEPROM.readFloat(16);
        v_p_2 = EEPROM.readFloat(20);
        v_i_2 = EEPROM.readFloat(24);
        motor.PID_velocity.P = v_p_1;      // 启动时使用 p1
        motor.PID_velocity.I = v_i_1;
    }
    //命令设置
    comm.add("TA",do_TA);
    comm.add("START",do_START);
    comm.add("MOTOR",do_MOTOR);
    comm.add("SV",do_SV);
    comm.add("SA",do_SA);

#if FLAG_V //电压模式
    comm.add("K11",do_K11);
    comm.add("K12",do_K12);
    comm.add("K13",do_K13);
    comm.add("K21",do_K21);
    comm.add("K22",do_K22);
    comm.add("K23",do_K23);
#else //速度模式
    comm.add("VP1",do_vp1);
    comm.add("VI1",do_vi1);
    comm.add("VP2",do_vp2);
    comm.add("VI2",do_vi2);
    comm.add("TV",do_tv);
    comm.add("K31",do_K31);
    comm.add("K32",do_K32);
    comm.add("K33",do_K33);
    comm.add("K41",do_K41);
    comm.add("K42",do_K42);
    comm.add("K43",do_K43);
#endif
    // kalman mpu6050 init
    Wire.begin(19, 18,400000);// Set I2C frequency to 400kHz
    i2cData[0] = 7;    // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g

    while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

    while (i2cRead(0x75, i2cData, 1));
    if (i2cData[0] != 0x68)
    { // Read "WHO_AM_I" register
        Serial.print(F("Error reading sensor"));
        while (1);
    }

    delay(100); // Wait for sensor to stabilize

    /* Set kalman and gyro starting angle */
    while (i2cRead(0x3B, i2cData, 6));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    double pitch = acc2rotation(accX, accY, kalAngleZ);

    kalmanZ.setAngle(pitch);
    gyroZangle = pitch;

    timer = micros();
    Serial.println("kalman mpu6050 init");

    //wifi初始化
    WiFi.mode(WIFI_AP);
    //启动AP
    while(!WiFi.softAP(ssid, password)); 
    Serial.println("AP启动成功");
    //等待udp监听设置成功
    while (!udp.listen(localUdpPort));
    
    udp.onPacket(onPacketCallBack); //注册收到数据包事件

    I2Ctwo.begin(23, 5, 400000);   //SDA,SCL
    sensor.init(&I2Ctwo);

    //连接motor对象与传感器对象
    motor.linkSensor(&sensor);

    //供电电压设置 [V]
    driver.voltage_power_supply = 12;
    driver.init();

    //连接电机和driver对象
    motor.linkDriver(&driver);

    //FOC模型选择
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    //运动控制模式设置
#if FLAG_V
    motor.controller = MotionControlType::torque;
#else
    motor.controller = MotionControlType::velocity;
    //速度PI环设置
    motor.PID_velocity.P = 0.5;
    motor.PID_velocity.I = 20;
#endif


    //最大电机限制电机
    motor.voltage_limit = 12;

    //速度低通滤波时间常数
    motor.LPF_velocity.Tf = 0.01;

    //设置最大速度限制
    motor.velocity_limit = 40;

    motor.useMonitoring(Serial);

    //初始化电机
    motor.init();

    //初始化 FOC
    motor.initFOC();

    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target velocity using serial terminal:"));

    //digitalWrite(22,LOW);  // 默认停止电机
}
char buf[255];
long loop_count = 0;
double last_pitch;

void loop() {
  
    motor.loopFOC();
//    loop_count++ == 10
//    loop_count = 0;
    while(i2cRead(0x3B, i2cData, 14));

    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);    // z 轴角速度 rad/s
    
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    
    double pitch = acc2rotation(accX, accY, kalAngleZ);
    double gyroZrate = gyroZ / 131.0; // Convert to deg/s 弧度每秒转换为度每秒
    if(abs(pitch-last_pitch)>100)
        kalmanZ.setAngle(pitch);
    
    kalAngleZ = kalmanZ.getAngle(pitch, gyroZrate + gyroZ_OFF, dt);
    last_pitch = pitch;
    gyroZangle += (gyroZrate + gyroZ_OFF) * dt;
    compAngleZ = 0.93 * (compAngleZ + (gyroZrate + gyroZ_OFF) * dt) + 0.07 * pitch; // 角度补偿

    // Reset the gyro angle when it has drifted too much
    if(gyroZangle < -180 || gyroZangle > 180)
        gyroZangle = kalAngleZ;
    
    // 对卡尔曼的值(0~360)以120取模，限制到0~120，从而控制出三个面的直立
    float pendulum_angle = constrainAngle(fmod(kalAngleZ,120) - target_angle);
//  FLAG_V为1时使用电压控制，为0时候速度控制
#if FLAG_V 
    if(abs(pendulum_angle) < swing_up_angle) // if angle small enough stabilize 0.5~30°,1.5~90°
    {
        target_voltage = controllerLQR(pendulum_angle, gyroZrate, motor.shaftVelocity());
        // limit the voltage set to the motor
        if(abs(target_voltage) > motor.voltage_limit * 0.7)
            target_voltage = _sign(target_voltage) * motor.voltage_limit * 0.7;
    }
    else // else do swing-up
    {    // sets 1.5V to the motor in order to swing up
        target_voltage = -_sign(gyroZrate) * swing_up_voltage;
    }

    // set the target voltage to the motor
    if (accZ < -13000 && ((accX * accX + accY * accY) > (14000 * 14000))) {
        motor.move(0);
    } else {
        motor.move(lpf_throttle(target_voltage));
    }

#else
    if (abs(pendulum_angle) < swing_up_angle) // if angle small enough stabilize 0.5~30°,1.5~90°
    {
        target_velocity = controllerLQR(pendulum_angle, gyroZrate, motor.shaftVelocity());
        if (abs(target_velocity) > 140)
            target_velocity = _sign(target_velocity) * 140;
        
        motor.controller = MotionControlType::velocity;
        motor.move(target_velocity);
    } else { 
        // else do swing-up
        // sets 1.5V to the motor in order to swing up
        motor.controller = MotionControlType::torque;
        target_voltage = -_sign(gyroZrate) * swing_up_voltage;
        motor.move(target_voltage);
    }
#endif

#if DUBUG
    // 串口调试打印
    Serial.print(pitch);Serial.print("\t");
    Serial.print(kalAngleZ);Serial.print("\t");
    Serial.print("\r\n");
#endif

    //可以使用该方法广播信息
    if(wifi_flag)
    {
        memset(buf, 0, strlen(buf));  

        wifi_print("v", motor.shaftVelocity()); // 电机转速
        wifi_print("vq", motor.voltage.q); // 电机的期望电压
        wifi_print("p", pendulum_angle); // 期望角度和真实角度的差值
        wifi_print("t", target_angle);   // 目标角度
        wifi_print("k", kalAngleZ);      // 卡尔曼滤波后的角度
        wifi_print("g", gyroZrate);      // z轴的加速度

        udp.writeTo((const unsigned char*)buf, strlen(buf), IPAddress(192,168,4,2), localUdpPort); //广播数据
    }
}


// function constraining the angle in between -60~60
// 本来是 -targetangle~120 
float constrainAngle(float x)
{
    float a = 0;
    if(x < 0) {
        a = 120 + x;
        if(a < abs(x))
            return a;
    }
    return x;
}
// LQR stabilization controller functions
// calculating the voltage that needs to be set to the motor in order to stabilize the pendulum
float controllerLQR(float p_angle, float p_vel, float m_vel)
{
    // if angle controllable
    // calculate the control law
    // LQR controller u = k*x
    //  - k = [40, 7, 0.3]
    //  - k = [13.3, 21, 0.3]
    //  - x = [pendulum angle, pendulum velocity, motor velocity]'

    if(abs(p_angle) > 2.5) {
        last_unstable_time = millis();
        if(stable) {
            target_angle = EEPROM.readFloat(0);
            stable = 0;
        }
    }
    if((millis() - last_unstable_time) > 1000&&!stable) {
        target_angle = target_angle+p_angle;
        stable = 1;
    }

    //Serial.println(stable);
    float u;
#if FLAG_V
    if (!stable) {
        u = LQR_K1_1 * p_angle + LQR_K1_2 * p_vel + LQR_K1_3 * m_vel;
    } else {
        //u = LQR_K1 * p_angle + LQR_K2 * p_vel + LQR_K3 * m_vel;
        u = LQR_K2_1 * p_angle + LQR_K2_2 * p_vel + LQR_K2_3 * m_vel;
    }
#else
    if(!stable) {
        motor.PID_velocity.P = v_p_1;
        motor.PID_velocity.I = v_i_1;
        u = LQR_K3_1 * p_angle + LQR_K3_2 * p_vel + LQR_K3_3 * m_vel;
    } else {
        motor.PID_velocity.P = v_p_2;
        motor.PID_velocity.I = v_i_2;
        //u = LQR_K1 * p_angle + LQR_K2 * p_vel + LQR_K3 * m_vel;
        u = LQR_K4_1 * p_angle + LQR_K4_2 * p_vel + LQR_K4_3 * m_vel;
    }
#endif

    return u;
}

void wifi_print(char * s,double num)
{
  char str[255];
  char n[255];
  sprintf(n, "%.2f",num);
  strcpy(str,s);
  strcat(str, n);
  strcat(buf+strlen(buf), str);
  strcat(buf, ",\0");
}
