#include "RolypolyController.h"
#include <string.h>

void RolypolyController::attitudeControl() {



}
void RolypolyController::AngularControl(double dt) {
	roll_err = roll_setPoint - m_roll;
	pitch_err = pitch_setPoint - m_pitch;
	yaw_err = yaw_setPoint - m_yaw;

	// P control
	roll_p = roll_err * p_gain;
	pitch_p = pitch_err * p_gain;
	yaw_p = yaw_err * p_gain_yaw;

	// I control
	roll_err_integral += roll_err * dt;
	roll_i = roll_err_integral * i_gain;
	pitch_err_integral += pitch_err * dt;
	pitch_i = pitch_err_integral * i_gain;
	yaw_err_integral += yaw_err * dt;
	yaw_i = yaw_err_integral * i_gain_yaw;

	// integral wind_up 
	if(roll_err_integral > i_wind_up_limit) { 
		roll_err_integral = i_wind_up_limit;
	}
	else if(roll_err_integral < 0-i_wind_up_limit) {
		roll_err_integral = 0-i_wind_up_limit;
	}
	if(pitch_err_integral > i_wind_up_limit) { 
		pitch_err_integral = i_wind_up_limit;
	}
	else if(pitch_err_integral < 0-i_wind_up_limit) {
		pitch_err_integral = 0-i_wind_up_limit;
	}
	if(yaw_err_integral > i_wind_up_limit_yaw) { 
		yaw_err_integral = i_wind_up_limit_yaw;
	}
	else if(yaw_err_integral < 0-i_wind_up_limit_yaw) {
		yaw_err_integral = 0-i_wind_up_limit_yaw;
	}

	// D control
	roll_d = gx_mean * d_gain / dt;	
	pitch_d = -gy_mean * d_gain / dt;
	yaw_d = gz_lpf * d_gain_yaw / dt;

	roll_pid = roll_p + roll_i - roll_d;
	pitch_pid = pitch_p  + pitch_i - pitch_d;
	yaw_pid = yaw_p + yaw_i + yaw_d;

	

	//tmp_pid = roll_p + roll_i + roll_d;
}
void RolypolyController::AngularRateControl() {
	
	// queue shift
	memcpy( &roll_pid_q[0], &roll_pid_q[1], sizeof(double)*(TIMED_MULTIPLEX_CONTROL_DELAY_COUNT-1));
	memcpy( &pitch_pid_q[0], &pitch_pid_q[1], sizeof(double)*(TIMED_MULTIPLEX_CONTROL_DELAY_COUNT-1));
	roll_pid_q[TIMED_MULTIPLEX_CONTROL_DELAY_COUNT-1] = roll_pid;
	pitch_pid_q[TIMED_MULTIPLEX_CONTROL_DELAY_COUNT-1] = pitch_pid;
	roll_pid_q_avg = 0;	pitch_pid_q_avg = 0;
	for(int i=0; i<TIMED_MULTIPLEX_CONTROL_DELAY_COUNT; i++) {
		roll_pid_q_avg += roll_pid_q[i];
		pitch_pid_q_avg += pitch_pid_q[i];
	}
	roll_pid_q_avg /= (double)TIMED_MULTIPLEX_CONTROL_DELAY_COUNT;
	pitch_pid_q_avg /= (double)TIMED_MULTIPLEX_CONTROL_DELAY_COUNT;

	//
	//roll_rate_err = roll_pid - gx_mean;
	//pitch_rate_err = pitch_pid - gy_mean;

	// 각속도 P 제어		// 부호 주의할 것!
	roll_rate_p = (roll_pid - gx_mean) * p_gain_rate;
	pitch_rate_p = (pitch_pid - gy_mean) * p_gain_rate;

	roll_rate_err = roll_pid_q_avg - gx_mean;
	pitch_rate_err = pitch_pid_q_avg - gy_mean;
	

	// 각속도 D제어
	roll_rate_d = (roll_rate_err - roll_rate_err_last) * d_gain_rate / m_dt;
	pitch_rate_d = (pitch_rate_err - pitch_rate_err_last) * d_gain_rate / m_dt;

	roll_rate_err_last = roll_rate_err;
	pitch_rate_err_last = pitch_rate_err;

	roll_rate_pid = roll_rate_p + roll_rate_d;
	pitch_rate_pid = pitch_rate_p + pitch_rate_d;

}

void RolypolyController::generatePWM(int* pwm1, int* dir1, int* pwm2, int* dir2) {
	
	if( pitch_rate_pid < 0 ) {
		*dir1 = 1;
		*dir2 = 1;
		*pwm1 = 255 + pitch_rate_pid;
		*pwm2 = 255 + pitch_rate_pid;
	}else {
		*dir1 = 0;
		*dir2 = 0;
		*pwm1 = pitch_rate_pid;
		*pwm2 = pitch_rate_pid;
	}




}

RolypolyController::RolypolyController() {
	initialize();

}

RolypolyController::~RolypolyController() {


}



void RolypolyController::initialize() {


	p_gain = 65.0;//2.0;//2.2;
	i_gain = 0;//0.3;
	d_gain = 0;//0.015;

	p_gain_rate = 1.70;//0.15;//0.17;//0.16;
	d_gain_rate = 0.0;

	p_gain_yaw = 0.18;//0.38;
	i_gain_yaw = 0;//0.10;
	d_gain_yaw = 0.0018;


	roll_p=0, roll_i=0, roll_d=0;
	pitch_p=0, pitch_i=0, pitch_d=0;
	yaw_p=0, yaw_i=0, yaw_d=0;

	roll_rate_p = 0, pitch_rate_p = 0;

	roll_setPoint = 0.0;
	pitch_setPoint = 0.0;
	yaw_setPoint = 0.0;
	roll_err = 0;
	pitch_err = 0;
	yaw_err = 0;
	roll_err_integral = 0; pitch_err_integral = 0; yaw_err_integral = 0;
	i_wind_up_limit = 5; i_wind_up_limit_yaw = 10;
	
	f_right = 0; f_left = 0; f_front = 0; f_back = 0;

	ctl_gain = 1.00;

	isFirst = true;

	tmp_pid = 0;

	roll_rate_p = 0;	roll_rate_d = 0;	roll_rate_pid = 0; 
	pitch_rate_p = 0;	pitch_rate_d = 0;	pitch_rate_pid = 0; 

}

void RolypolyController::destroy() {

}
void RolypolyController::setEulerAngle(double roll, double pitch, double yaw) {
	m_roll = roll;
	m_pitch = pitch;
	m_yaw = yaw;
}
void RolypolyController::setSmoothValue(double xValue, double yValue, double zValue) {
	gx_mean = xValue;
	gy_mean = yValue;
	gz_lpf = zValue;

}
void RolypolyController::setSetPoint(double r, double p) {
	roll_setPoint = r;
	pitch_setPoint = p;


}