/*
 * main.cpp
 *
 *  Created on: 2022/04/06
 *      Author: Antonio Gallina
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "pid.h"

// Constants
#define TRANSMISSION_RATIO 		28.0			//
#define STEER_DEG2PWM_RATIO 	0.0009505		// [deg/pwm]
//#define STEER_DEG2PWM_OFFSET	0.07525			// [pwm]: original in the Embedded platform
#define STEER_DEG2PWM_OFFSET	0.07620			// [pwm]
#define M_PPI					6.28318530718	//
#define WHEEL_RADIUS			0.03			// [m]
#define TS_ENCODER_SPEED_PUB	10				// [ms]
#define MAX_FORWARD_SPEED		1.1				// [m/s]
#define MAX_BACKWARD_SPEED		-0.5			// [m/s]
#define MAX_STEERING_ANGLE		28				// [deg]
#define MOT_DECELERATION		-2				// [m/ss]
#define MOT_ACCELERATION		2				// [m/ss]

#define SPEED_FFW				0				//
#define EPSILON					0.01			//
#define EPSILON_DIFF			0.0001			//

//extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim16;

ros::NodeHandle nh;

// REFERENCE variables
double speedTarget=0.0, speedTargetPos=0.0, speedRef=0.0;// [m/s], [m/s]: speed references
double steerRef=0.0;// [deg]: steer references
// MEASUREMENT variables
double wheelSpeed=0.0, motorSpeed=0.0, carSpeed=0.0;// [rps], [rps], [m/s]: speed measurements
double globalDistance=0.0, localDistance=0.0, localDistanceOrigin=0.0, localDistanceRef=0.0;// [m]: position measurements
long encoderCount=0;// [tick]: encoder count
// CONTROL variables
double motorPWM=0.0, servoPWM=0.0;// [tick], [tick]: control variables
// STATE of the CAR
bool isSpeedControl 	= false;
bool isPositionControl 	= false;

bool posAckDone=false, reachedPosition=false, pub_flag=false;

// CONTROLLERS
PID SpeedController = PID(&carSpeed, &motorPWM, &speedRef, 5.0, 8.0, 0.001, _PID_P_ON_E, _PID_CD_DIRECT);// PID_Proportional_On_Error
PID PositionController = PID(&localDistance, &speedTargetPos, &localDistanceRef, 6.0, 0.0, 4.0, _PID_P_ON_E, _PID_CD_DIRECT);// PID_Proportional_On_Error


// ROS STUFF
std_msgs::Float32 carSpeed_msg;
std_msgs::Float32 globalDistance_msg;
std_msgs::Bool posFeedback_msg;
// command callbacks
void command_speed_callback(const std_msgs::Float32& msg);
void command_steer_callback(const std_msgs::Float32& msg);
void command_stop_callback(const std_msgs::Float32& msg);
void command_pos_callback(const std_msgs::Float32& msg);

void stop(float angle);
void steer(float angle);
void speed(float vel);
void position(float dist);
void drive_pwm(float pwm_value);

bool isClose(double A, double B);
double last_diff=0.0;

// publishers and subscribers
ros::Publisher carSpeed_pub("automobile/encoder/speed", &carSpeed_msg);
ros::Publisher globalDistance_pub("automobile/encoder/distance", &globalDistance_msg);
ros::Publisher posFeedback_pub("automobile/feedback/position", &posFeedback_msg);
ros::Subscriber<std_msgs::Float32> speed_sub("automobile/command/speed", &command_speed_callback);
ros::Subscriber<std_msgs::Float32> steer_sub("automobile/command/steer", &command_steer_callback);
ros::Subscriber<std_msgs::Float32> stop_sub("automobile/command/stop", &command_stop_callback);
ros::Subscriber<std_msgs::Float32> pos_sub("automobile/command/position", &command_pos_callback);


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	nh.getHardware()->reset_rbuf();
}


void command_speed_callback(const std_msgs::Float32& msg){
	speed(msg.data);
}

void command_steer_callback(const std_msgs::Float32& msg){
	steer(msg.data);
}

void command_stop_callback(const std_msgs::Float32& msg){
	stop(msg.data);
}

void command_pos_callback(const std_msgs::Float32& msg){
	position(msg.data);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim16){//every 1ms
		double DT = ((ENC_COUNT_MAX + 1) / 1e6);
		// ---------- ENCODER SPEED ----------
		encoderCount = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);// cast to [-2^15 ; 2^15]
	    motorSpeed = (encoderCount / 1024.0) / DT;// [rps]: motor speed
	    wheelSpeed = motorSpeed / TRANSMISSION_RATIO;// [rps]: wheel speed
	    carSpeed = wheelSpeed * M_PPI * WHEEL_RADIUS;// [m/s]: carSpeed, i.e. v = omega * R

	    globalDistance = globalDistance + carSpeed * DT;// [m]: signed distance run by the car, i.e. curvilinear abscissa
	    localDistance = globalDistance - localDistanceOrigin;// [m]: signed relative distance run by the car, i.e. curvilinear abscissa

	    // ---------- DOUBLE LOOP CONTROL ----------
		// ---------- OUTER LOOP: POSITION CONTROL - PID ----------
		if(isPositionControl){
			if(isClose(localDistance, localDistanceRef) and not posAckDone){
				reachedPosition = true;
				speedTarget = 0.0;
			}
			else{
				reachedPosition = false;
				PositionController.Compute();
				speedTarget = speedTargetPos;
			}
		}

		// ---------- INNER LOOP: SPEED CONTROL - PID ----------
		if(isSpeedControl){
			if(speedRef > speedTarget){
				speedRef = speedRef + MOT_DECELERATION*DT;
			}
			else if(speedRef < speedTarget){
				speedRef = speedRef + MOT_ACCELERATION*DT;
			}
			else{}

			SpeedController.Compute();// SPEED control action
		}

		// ---------- ACTUATION - SPEED ----------
		drive_pwm(motorPWM);//+ SPEED_FFW*speedRef; //FEEDFORWARD control action

	    // ---------- RESET ENCODER COUNTER ----------
		__HAL_TIM_SET_COUNTER(&htim4,0);
	}
	if(htim == &htim6){//every 10ms
		pub_flag = true;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	// ---------- USER BUTTON EMERGENCY BRAKE ----------
	if(GPIO_Pin == B1_Pin){
		stop(0.0);
	}
}

void setup(void)
{
	// Initialize the node ...
	nh.initNode();
	// ... advertise publishers ...
	nh.advertise(carSpeed_pub);
	nh.advertise(globalDistance_pub);
	nh.advertise(posFeedback_pub);
	// ... and declare subscribers
	nh.subscribe(speed_sub);
	nh.subscribe(steer_sub);
	nh.subscribe(stop_sub);
	nh.subscribe(pos_sub);

	// Set PID SPEED controller parameters
	SpeedController.SetMode(_PID_MODE_AUTOMATIC);// AUTOMATIC MODE
	SpeedController.SetOutputLimits(-0.3, 0.3);// PWM Output limits
	SpeedController.SetSampleTime((ENC_COUNT_MAX + 1) / 1e3);// set sample time in [ms]

	// Set PID POSITION controller parameters
	PositionController.SetMode(_PID_MODE_AUTOMATIC);// AUTOMATIC MODE
	PositionController.SetOutputLimits(-0.2, 0.2);// speed output limits
	PositionController.SetSampleTime((ENC_COUNT_MAX + 1) / 1e3);// set sample time in [ms]

	// Start TIMERS
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);// DC MOTOR PWM
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);// SERVO PWM
	HAL_TIM_Base_Start(&htim4);// ENCODER COUNTER
	HAL_TIM_Base_Start_IT(&htim16);// ENCODER SPEED AND SPEED CONTROL
	HAL_TIM_Base_Start_IT(&htim6);// PUBLISH TIMER

	// stop the car
	speedTarget = 0.0;
	speedRef 	= 0.0;
	steerRef 	= 0.0;
	stop(steerRef);

}

void loop(void)
{
	if(nh.connected()){
		if(pub_flag){
			// ---------- PUBLISH ----------
			carSpeed_msg.data = carSpeed;
			carSpeed_pub.publish(&carSpeed_msg);

			globalDistance_msg.data = globalDistance;
			globalDistance_pub.publish(&globalDistance_msg);

			if(reachedPosition and not posAckDone){
				posFeedback_msg.data = reachedPosition;
				posFeedback_pub.publish(&posFeedback_msg);
				posAckDone = true;
			}

			pub_flag = false;
		}

	}
	HAL_Delay(1);
	nh.spinOnce();
}

void stop(float angle){
	// ---------- STATE ----------
	isPositionControl 	= false;
	isSpeedControl 		= false;

	// ---------- STOP references ----------
	speedTarget = 0.0;
	speedTargetPos = 0.0;
	speedRef = 0.0;


	// ---------- STOP ----------
	motorPWM = 0.0;
	steer(angle);
}

void speed(float vel){
	// ---------- SPEED references ----------
	if(vel > MAX_FORWARD_SPEED){speedTarget = MAX_FORWARD_SPEED;}
	else if (vel < MAX_BACKWARD_SPEED){speedTarget = MAX_BACKWARD_SPEED;}
	else{speedTarget = vel;}

	// ---------- STATE ----------
	isSpeedControl 		= true;
	isPositionControl 	= false;
}

void steer(float angle){
	// ---------- STATE ----------

	// ---------- STEER references ----------
	if(angle > MAX_STEERING_ANGLE){steerRef = MAX_STEERING_ANGLE;}
	else if (angle < -MAX_STEERING_ANGLE){steerRef = -MAX_STEERING_ANGLE;}
	else{steerRef = angle;}

	// ---------- STEER ----------
	servoPWM = STEER_DEG2PWM_RATIO * steerRef + STEER_DEG2PWM_OFFSET;// [deg] -> [pwm]
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, servoPWM*SERVO_PWM_COUNT_MAX);// set servo pwm
}

void position(float dist){
	// ---------- POSITION references ----------
	localDistanceOrigin = globalDistance;
	localDistanceRef = dist;

	// ---------- STATE ----------
	isSpeedControl = true;
	isPositionControl = true;
	posAckDone = false;
}

void drive_pwm(float pwm_value){
	// ---------- ACTUATION - SPEED ----------
	if(pwm_value > 0.05){// FORWARDS
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);// IN_A -> 1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);// IN_B -> 0
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, abs(pwm_value)*MOT_PWM_COUNT_MAX);// set dc motor pwm
	}
	else if(pwm_value < -0.05){// BACKWARDS
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);// IN_A -> 0
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);// IN_B -> 1
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, abs(pwm_value)*MOT_PWM_COUNT_MAX);// set dc motor pwm
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);// IN_A -> 0
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);// IN_B -> 0
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);// set dc motor pwm
	}

}

bool isClose(double A, double B)
{
   double diff = A - B;
   return (diff < EPSILON) && (-diff < EPSILON) && (abs(last_diff-diff) < EPSILON_DIFF) ;
   last_diff = diff;
}

