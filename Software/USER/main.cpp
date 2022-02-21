#include "Arduino.h"
#include <SimpleFOC.h>
#include "SPI.h"
//-2.92 3.35
#define LED_PIN PC13
//5.8715      1.48
//motor.PID_velocity.P = 0.1;
//  motor.PID_velocity.I = 1.5;
//  motor.PID_velocity.D = 0.000003;
//  motor.LPF_velocity.Tf = 0.05;
//  motor.P_angle.P = 14;
//  motor.velocity_limit = 50;
//  //motor.voltage_limit = 4;
float target_angle = 5.8915; //摆平衡时的角度，手动测量，这个角度需要归一化，也就是0-2*PI，最好安装磁编码器的时候让这个值不要太接近0或者6.28,后边的代码没有处理边界情况
float bar_angle = 0;
float up_angle,costheta;
float angle_kp,angle_ki,angle_kd;
float threshold = 5;
float total_error=0;
float error=0;
float bar_vel,bar_vel_filtered;
float dead_force =0.0;// 0.05;
float base_vel,base_angle,base_vel_filtered;

float theta,theta_dot;
float last_force;

float abs_pos,off;

int state=1;
bool first_enter=true;
int i=0;

LowPassFilter filter = LowPassFilter(0.001);
LowPassFilter base_filter = LowPassFilter(0.01);

// velocity set point variable
float target_force = 0;

/**
 *
 * Position/angle motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target angle (in radians) from serial terminal
 *
 */


// magnetic sensor instance - SPI
//MagneticSensorSPI sensor = MagneticSensorSPI(MA730_SPI, PA4);

MagneticSensorSPI sensor = MagneticSensorSPI(MA730_SPI, PA4);
MagneticSensorSPI ma730 = MagneticSensorSPI(MA730_SPI, PB12);
//SPIClass SPI_2(SPI2);
// magnetic sensor instance - MagneticSensorI2C
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);


// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10, PA0);

// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// angle set point variable
//float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial2);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void setup() {
	angle_kp = 1.4;//-2.52;//-10;
  angle_kd =-0.1;//-0.15;// -150;
  angle_ki = 0.0053;
	
  pinMode(PC13, OUTPUT);
  // initialise magnetic sensor hardware
  sensor.init(&SPI);
	ma730.init(&SPI_2);
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
	
 // aligning voltage [V]
  motor.voltage_sensor_align = 3;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

// velocity PI controller parameters
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  
  
  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;
	motor.PID_velocity.output_ramp = 1000;
  // contoller configuration
  // default parameters in defaults.h
  // velocity PI controller parameters
//  motor.PID_velocity.P = 0.1;
//  motor.PID_velocity.I = 1.5;
//  motor.PID_velocity.D = 0.0000003;
//  motor.LPF_velocity.Tf = 0.05;
//  motor.P_angle.P = 14;
motor.velocity_limit = 50;
//  //motor.voltage_limit = 4;

  // velocity low pass filtering time constant
  // the lower the less filtered
  //motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  //motor.P_angle.P = 20;
  // maximal velocity of the position control
  //motor.velocity_limit = 20;

  // use monitoring with serial
  Serial2.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial2);

  motor.sensor_direction=Direction::CW;

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();
  
  off=motor.zero_electric_angle/7.0;

  // add target command T
  command.add('T', doTarget, "target angle");

  Serial2.println(F("Motor ready."));
  Serial2.println(F("Set the target angle using serial terminal:"));
  _delay(500);
	  //取一下前面的数据，不然滤波可能有问题
  for(int j=0;j<1000;j++){
    bar_angle = ma730.getAngle();
    bar_vel=ma730.getVelocity();
    base_angle=sensor.getAngle();
    base_vel = sensor.getVelocity();
    bar_vel_filtered=filter(bar_vel);
    base_vel_filtered=base_filter(base_vel); 
  }
		Serial2.println("state,error,vel,f_vel,base_vel,angle");

}


void loop() {
	 
	ma730.update();
	sensor.update();

//对于摆来说，磁钢在安装的时候可能方向不一样，所以摆的角度的正方向不确定，需要调整角度和速度的方向为逆时针(正视图)
 //对于基座来说，由于光电编码器的AB两线和单片机的连接顺序不定，基座的正方向也不确定，需要调整角度和速度的方向为逆时针(俯视图)
 //需要根据正方向自行调节下面的代码
  bar_angle = -ma730.getAngle();
  bar_vel = -ma730.getVelocity();
  base_vel = -sensor.getVelocity();

  bar_vel_filtered=filter(bar_vel); 
  base_vel_filtered=base_filter(base_vel);
  //abs_pos=-off+encoder.getAngle(); //校准时候用的，先不用
  error=_normalizeAngle(bar_angle)-target_angle;


  theta=3.1415+error; //theta是定义为摆的角度，摆竖直向下的时候为0，逆时针为正
  theta_dot=bar_vel_filtered;
  costheta=cos(theta); //提前计算，方便后边使用
	digitalWrite(PC13, LOW);
  //一个小状态机，一共四个状态
  switch(state){
    case 1: //idle 空状态，当摆竖直朝下的时候，给一个脉冲，不然energy shaping法无法启动，摆不能自己摆起来
      if(i<100){
        _delay(1);
        target_force=0.8; 
        i++;
      }
      else
        state=2;
      break;
    case 2: //swing up 使用energy shaping法摆起来，接近平衡位置的时候切换状态到LQR
      target_force = 0.0035*theta_dot*costheta*(-8.5*(1+costheta)+0.0075*theta_dot*theta_dot); 
      if(abs(error)<0.1)
        state=3;
      break;
    case 3: //LQR 这里的参数是手动调的，结构上使用的是LQR的结构
			digitalWrite(PC13, HIGH);
			//target_force =base_vel_filtered*0.05-1.5*error-0.1*bar_vel_filtered;
      target_force =base_vel_filtered*0.05*angle_kp-1.5*error*angle_kp-0.1*bar_vel_filtered*angle_kp;
      if(abs(error)>0.25)
        state=4;
      break;
    case 4: //swing down 就是Swing up的逆向使用，为了更快的让摆竖直向下
      target_force = -0.007*theta_dot*costheta*costheta*costheta*costheta*(-8.5*(1+costheta)+0.0075*theta_dot*theta_dot);
		if(abs(theta)<0.2&&abs(theta_dot)<0.2){
        state=1;
        i=0;
      }
        
      break;
  }
//如果力矩方向和基座方向不一致，需要取反
target_force=-target_force;

  //死区补偿
  if(target_force>threshold)
    target_force=threshold;
  else if(target_force<-threshold)
    target_force=-threshold;
	
	Serial2.print(state); Serial2.print('\t');
  Serial2.print(error); Serial2.print('\t');
  Serial2.print(bar_vel); Serial2.print('\t');
  Serial2.print(bar_vel_filtered); Serial2.print('\t');
  Serial2.print(base_vel_filtered); Serial2.print('\t');
	Serial2.println(_normalizeAngle(bar_angle));
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();
 // Serial2.println(sensor.getAngle());
// _delay(10);
  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
   //注释掉了校准力矩
  //target_force+=calib[(int)(_normalizeAngle(abs_pos)/0.00628)];  
  if(target_force>0)target_force+=dead_force;
  else if(target_force<0)target_force-=dead_force;
  //转速过大就保护
  if(abs(base_vel_filtered)>50) target_force=0;
  motor.move(target_force);
	
//  Serial2.print(target_force,5); Serial2.print('\t');
//  Serial2.print(ma730.getAngle());  Serial2.print('\t');
//	Serial2.println(sensor.getAngle());
  //Serial2.print("\t");
//	Serial2.println(ma730.getVelocity());

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();
  // user communication
  //command.run();
}

/**
  * @brief  Main Function
  * @param  None
  * @retval None
  */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    GPIO_JTAG_Disable();
    //SysClock_Init(F_CPU_128MHz);
    Delay_Init();
    ADCx_Init(ADC1);
    setup();
    for(;;)loop();
}

