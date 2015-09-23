#include "RolypolyFilter.h"
#include <math.h>

#define abs(x) ((x)>0?(x):-(x))

void RolypolyFilter::setRawData(int ax, int ay, int az, int gx, int gy, int gz) {
	
	// digital -> [g]	// mpu6050
	//m_ax = (double)ax / 16384.0;
	//m_ay = (double)ay / 16384.0;
	//m_az = (double)az / 16384.0;

	// digital -> [g]	// BMA150
	m_ax = ((double)-ax / 256.0) * scale_factor;
	m_ay = ((double)-ay / 256.0) * scale_factor;
	m_az = ((double)az / 256.0) * scale_factor;


	// digital -> [deg/s]
	m_gx = (double)gx * 0.0152;	// 0.015259 = 1 / (32768.0*500.0)
	m_gy = (double)gy * 0.0152;
	m_gz = (double)gz * 0.0152;

}

void RolypolyFilter::complementaryFilter() {

	m_roll_rad = m_roll/57.3;
	m_pitch_rad = m_pitch/57.3;

	roll_acc = atan( m_ay / sqrt(m_ax*m_ax+m_az*m_az) ) * 57.3;	// 57.3 = 180.0 / 3.141592
	roll_gyro_dot = m_gx + m_gy*sin(m_roll_rad)*tan(m_pitch_rad) + m_gz*cos(m_roll_rad)*tan(m_pitch_rad);
	roll_gyro += roll_gyro_dot * m_dt;
	//integral_gx += m_gx * m_dt;
	//integral_gx_trapez += ((( -m_gx*0.01526 ) + ( -m_gx1*0.01526 ))  * m_dt) / 2.0;
	cf_err_x = m_roll - roll_acc;
	cf_err_sum_x += cf_err_x * m_dt;
	cf_kp_val_x = cf_err_x * cf_kp;
	//cf_ki_val_x = cf_err_sum_x * cf_ki;
	cf_pid_out_x = roll_gyro_dot - (cf_kp_val_x);
	m_roll += cf_pid_out_x * m_dt;


	pitch_acc = -atan( m_ax / sqrt(m_ay*m_ay+m_az*m_az) ) * 57.3;	// 57.3 = 180.0 / 3.141592
	pitch_gyro_dot = m_gy*cos(m_roll_rad) - m_gz*sin(m_roll_rad);
	pitch_gyro += pitch_gyro_dot * m_dt;
	//integral_gy += m_gy * m_dt;
	cf_err_y = m_pitch - pitch_acc;
	cf_err_sum_y += cf_err_y * m_dt;
	cf_kp_val_y = cf_err_y * cf_kp;
	//cf_ki_val_y = cf_err_sum_y * cf_ki;
	cf_pid_out_y = pitch_gyro_dot - (cf_kp_val_y);
	m_pitch += cf_pid_out_y * m_dt;


	//m_gx1 = m_gx;

}

void RolypolyFilter::integralYaw() {

	m_yaw += -m_gz * m_dt;	// 0.15259 = 32768.0*500.0
	
	//yaw_gyro_dot = -(m_gy*(sin(m_roll_rad)/cos(m_pitch_rad)) + m_gz*(cos(m_roll_rad)/cos(m_pitch_rad)));
	//yaw_gyro += yaw_gyro_dot * m_dt;

	// Yaw 틸트 보상 알고리즘
	//double scaled_by_roll = m_gz * ((1/pow(42, 0.6))*pow(abs(m_roll), 1.6) / 42 * 0.32 + 1);
	//compensated_by_pitch = scaled_by_roll + scaled_by_roll*(1/pow(90, 0.6))*pow(abs(m_pitch), 1.6) / 90.0;
	//yaw_output += compensated_by_pitch * m_dt;
	
}


void RolypolyFilter::estimateVelbyAccel() {

	zeroG_ax = m_ax - (-sin( m_pitch_rad ));
	zeroG_ay = m_ay - (cos( m_pitch_rad )  * sin( m_roll_rad ));
	zeroG_az = m_az - (cos( m_pitch_rad )  * cos( m_roll_rad ));

	vel_xa = vel_xa + zeroG_ax * m_dt;
	vel_ya = vel_ya + zeroG_ay * m_dt;
	vel_za = vel_za + zeroG_az * m_dt;

	//pos_x = pos_x + vel_x * m_dt;
	//pos_y = pos_y + vel_y * m_dt;

}

void RolypolyFilter::estimateVelbyGyro() {

	// 방향과 크기 고려하지 않았음 // 방향은 차후에 가속도센서와 비교 후 맞추기
	vel_xg = vel_xg - (m_pitch/weight_body * m_dt);
	vel_yg = vel_yg + (m_roll/weight_body * m_dt);

}
void RolypolyFilter::accelerometerHSR() {

	

	abs_vector = sqrt(m_ax*m_ax + m_ay*m_ay + m_az*m_az);

	if( abs_vector > 1 ) {
		scale_factor = scale_factor / 1.0001;
	}else if( abs_vector < 1 ) {
		scale_factor = scale_factor * 1.0001;
	}

	//m_ax *= scale_factor;
	//m_ay *= scale_factor;
	//m_az *= scale_factor;


}

double RolypolyFilter::alt_filter(int ultra, int length) {
	alt_avg = 0;
	alt_sum = 0;

	for(int i=length+1; i>0; i--) {		// 원래는 length-1이지만 넉넉하게 +2 하자
		alt[i] = alt[i-1];
		if( i < length ) {
			alt_sum += alt[i];
		}
	}
	
	alt[0] = ultra;
	alt_sum += alt[0];

	alt_avg = (double)alt_sum / (double)(length);

	
	return alt_avg;

}

double RolypolyFilter::generalLowPassFilter(double* lpf, double data, double a, double b) {

	*lpf = a * *lpf + b * data;
	return *lpf;

}

double RolypolyFilter::generalMeanFilter(double* q, double len, double data) {

	double sum = 0;

	for(int i=0; i<len-1; i++) {
		q[i+1] = q[i];
	}
	
	q[0] = data;
	
	for(int i=0; i<len; i++) {
		sum += q[i];
	}

	return sum / len;

}


RolypolyFilter::RolypolyFilter() {
	initialize(0, 0, 255);

}

RolypolyFilter::~RolypolyFilter() {


}

void RolypolyFilter::initialize(int ax, int ay, int az) {
	cf_kp = 0.1;
	cf_ki = 0.0;
	cf_err_x = 0; cf_err_sum_x = 0; cf_err_y = 0; cf_err_sum_y = 0;
	cf_kp_val_x = 0; cf_ki_val_x = 0; cf_kd_val_x = 0; cf_kp_val_y = 0; cf_ki_val_y = 0; cf_kd_val_y = 0;
	cf_pid_out_x = 0; cf_pid_out_y = 0;
	integral_gx = 0; integral_gy = 0;
	compensated_by_pitch=0;
	yaw_output=0;
	m_roll = 0;
	m_pitch = 0;
	m_yaw = 0;
	for( int i=0; i<MEAN_Q_LENGTH; i++ ) {
		mean_q_gx[i] = 0;
		mean_q_gy[i] = 0;
	}
	prevPhi = 0;
	prevTheta = 0;
	prevPsi = 0;

	for( int i=0; i<ALT_Q_LENGTH; i++ ) {
		alt[i] = 0;
	}
	alt_sum = 0;	
	alt_avg = 0;


	roll_gyro = 0;
	pitch_gyro = 0;
	yaw_gyro = 0;

	//m_gx1 = 0;
	isFirst = true;

////// cf value init.
	// digital -> [g]	// BMA150
	m_ax = (double)-ax / 256.0;
	m_ay = (double)-ay / 256.0;
	m_az = (double)az / 256.0;
	m_roll = atan( m_ay / sqrt(m_ax*m_ax+m_az*m_az) ) * 57.3;
	m_pitch = -atan( m_ax / sqrt(m_ay*m_ay+m_az*m_az) ) * 57.3;

	vel_xa = 0; vel_ya = 0; vel_za = 0;
	//pos_x = 0; pos_y = 0;

	abs_vector = 0;
	scale_factor = 1;
}

void RolypolyFilter::destroy() {

}
