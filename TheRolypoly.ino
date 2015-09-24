#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "BMA150.h"
#include "RolypolyFilter.h"
#include "RolypolyController.h"
#include "RolypolyConfig.h"


int Rolypoly_state = 0;	// state management cariable



struct Rolypoly_attitude {
	double roll;
	double pitch;
	double yaw;
};

// bias calibration variable
double gx_bias = 0, gy_bias = 0, gz_bias = 0;
double ax_bias = 0, ay_bias = 0, az_bias = 0;

// timming variable
long start_time = 0, end_time = 0, loop_time = 0;
double elapsed_time = 0;
double dt = 0;
double dt_4angular = 0, cnt_4angular = 0;
long loop_cnt=0;

double roll_setPoint = 0, pitch_setPoint = 0, yaw_setpoint = 0;
double roll_sp_lpf = roll_setPoint, pitch_sp_lpf = pitch_setPoint, yaw_sp_lpf = yaw_setpoint;
double roll_offset = 0.0, pitch_offset = 2.6;


MPU6050 mpu6050 = MPU6050();
int16_t gx, gy, gz;

BMA150 bma150;
int16_t bma150_x, bma150_y, bma150_z;

struct Rolypoly_attitude RP_att;

RolypolyFilter RP_filter = RolypolyFilter();
RolypolyController RP_ctrler = RolypolyController();

bool led = true;

// PWM variable
int pwm1 = 0, pwm2 = 0;
int dir1 = 0, dir2 = 0;

volatile long pulse_right = 0, pulse_left = 0;
volatile long pulse_right_prev = 0, pulse_left_prev = 0;
double rps_right = 0, rps_left = 0;


void encoder_counter_right() { 
	if( digitalRead(ENCODER_RIGHT_B) ) {
		pulse_right--; 
	}else {
		pulse_right++; 
	}
}

void encoder_counter_left() { 
	if( digitalRead(ENCODER_LEFT_B) ) {
		pulse_left--; 
	}else {
		pulse_left++; 
	}
}

void setup() {
	Serial.begin(115200);
	
	Wire.begin();

	attachInterrupt(0, encoder_counter_right, RISING);
	attachInterrupt(5, encoder_counter_left, RISING);

	pinMode (MOTOR1_ENABLE_PIN, OUTPUT);
	pinMode(MOTOR1_PWM_PIN, OUTPUT);
	pinMode(MOTOR1_DIRECTION_PIN, OUTPUT);

	pinMode (MOTOR2_ENABLE_PIN, OUTPUT);
	pinMode(MOTOR2_PWM_PIN, OUTPUT);
	pinMode(MOTOR1_DIRECTION_PIN, OUTPUT);

	pinMode(13, OUTPUT);

	Serial.println("Initialize Sensors...");
	
	////////////////////////////////////////////////////////////////
	//					SENSOR INITIALIZE
	mpu6050.initialize();
	mpu6050.setI2CMasterModeEnabled(0);
	mpu6050.setI2CBypassEnabled(1);
	mpu6050.setDLPFMode(MPU6050_DLPF_BW_5);
	delay(5);

	bma150.initialize();

	////////////////////////////////////////////////////////////////
	//					BIAS MEASUREMENT
	Serial.println("Bias Measurment Step. Please Don't Move..");
	for(int i=1; i<=1000; i++) {
		mpu6050.getRotation(&gx, &gy, &gz);
		//Serial.println(gx);
		bma150.getAcceleration(&bma150_x, &bma150_y, &bma150_z);

		gx_bias += gx/1000.0;
		gy_bias += gy/1000.0;
		gz_bias += gz/1000.0;

		ax_bias += bma150_x/1000.0;
		ay_bias += bma150_y/1000.0;
		az_bias += (bma150_z-256)/1000.0;

	}

	memcpy(&RP_att, 0, sizeof(RP_att));

	//RolypolyFilter.initialize(bma150_x-ax_bias, bma150_y-ay_bias, bma150_z-az_bias); 
	RP_filter.initialize(bma150_x, bma150_y, bma150_z); 





	start_time = end_time = micros();


}

void loop() {
	// Timming
	end_time = micros();
	loop_time = end_time - start_time;
	if( loop_time < LOOP_TIME ) {
		delayMicroseconds(LOOP_TIME - loop_time);
		loop_time = LOOP_TIME;
	}
	start_time = micros();
	dt = loop_time / 1000000.0;
	elapsed_time += dt;


	if( Serial.available() ) {
		char cmd = Serial.read();
		if( cmd == 'a' ) {
			
		}

	}

//	Read raw accel/gyro measurements from device
	//mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	mpu6050.getRotation(&gx, &gy, &gz);
	if( gx==0 && gy==0 && gz==0 ) {
		mpu6050.getRotation(&gx, &gy, &gz);
	}

	bma150.getAcceleration(&bma150_x, &bma150_y, &bma150_z);

//	Apply Calibration
	//gx -= gx_bias;
	//gy -= gy_bias;
	//gz -= gz_bias;
	gx -= -93;
	gy -= 18;
	gz -= 7;

	//bma150_x -= (int)ax_bias;
	//bma150_y -= (int)ay_bias;
	//bma150_z -= (int)az_bias;

	RP_filter.setRawData(bma150_x, bma150_y, bma150_z, gx, gy, gz);
	RP_filter.setDt(dt);
	RP_filter.integralYaw();


	if(	cnt_4angular >= TIMED_MULTIPLEX_CONTROL_DELAY_COUNT ) {
		RP_filter.setDt(dt_4angular);
		RP_filter.complementaryFilter();
		RP_filter.accelerometerHSR();
		//RP_filter.estimateVelbyAccel();
		//RP_filter.estimateVelbyGyro();
		
	}
	
	RP_ctrler.setDt(dt);
	RP_ctrler.setSmoothValue(RP_filter.m_gx, RP_filter.m_gy, RP_filter.m_gz);

	if(	cnt_4angular >= TIMED_MULTIPLEX_CONTROL_DELAY_COUNT ) {
		RP_ctrler.setEulerAngle(RP_filter.m_roll - roll_offset, RP_filter.m_pitch - pitch_offset, RP_filter.m_yaw);
		//roll_sp_lpf = roll_sp_lpf * 0.5 + roll_setPoint * 0.5;
		//pitch_sp_lpf = pitch_sp_lpf * 0.5 + pitch_setPoint * 0.5;
		RP_ctrler.setSetPoint( roll_setPoint, pitch_setPoint );
		RP_ctrler.AngularControl(dt_4angular);
	}

	RP_ctrler.AngularRateControl();
	RP_ctrler.generatePWM(&pwm1, &dir1, &pwm2, &dir2);
	
	if(	cnt_4angular >= TIMED_MULTIPLEX_CONTROL_DELAY_COUNT ) {


		dt_4angular = 0;
		cnt_4angular = 0;

	}
	dt_4angular += dt;
	cnt_4angular ++;
	
	/////////////////////////////////////// 자세 각도제어 끝
	/////////////////////////////////////// 모터 속도제어 시작

	rps_right = ( (pulse_right - pulse_right_prev) / 11356.0) / 0.01;
	rps_left = ( (pulse_left - pulse_left_prev) / 11356.0) / 0.01;
	



	pulse_right_prev = pulse_right;
	pulse_left_prev = pulse_left;



	/////////////////////////////////////// 모터 속도제어 끝


	digitalWrite(MOTOR1_ENABLE_PIN, 1);
	digitalWrite(MOTOR1_DIRECTION_PIN, dir1);
	analogWrite(MOTOR1_PWM_PIN, (int)pwm1);

	digitalWrite(MOTOR2_ENABLE_PIN, 1);
	digitalWrite(MOTOR2_DIRECTION_PIN, dir2);
	analogWrite(MOTOR2_PWM_PIN, (int)pwm2);


	loop_cnt++;

	Serial.print(RP_filter.pitch_acc);
	Serial.print("\t");
	Serial.print(RP_filter.pitch_gyro);
	Serial.print("\t");
	Serial.print(RP_filter.m_pitch);
	Serial.println();

	if(loop_cnt > 10) {
		
		Serial.print(rps_right);
		Serial.print("\t");
		Serial.print(rps_left);
		Serial.println();
		
		//led = !led;
		//digitalWrite(13, led);
		loop_cnt = 0;
	}

}
