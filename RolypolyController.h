#include "RolypolyConfig.h"

#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

class RolypolyController {

public:
	double m_roll, m_pitch, m_yaw;

	double p_gain, i_gain, d_gain;
	double p_gain_rate, d_gain_rate;
	double roll_p, roll_i, roll_d, roll_pid;
	double pitch_p, pitch_i, pitch_d, pitch_pid;
	double yaw_p, yaw_i, yaw_d, yaw_pid;
	double roll_rate_err, pitch_rate_err;
	double roll_rate_err_last, pitch_rate_err_last;
	double roll_rate_p, roll_rate_d, roll_rate_pid;
	double pitch_rate_p, pitch_rate_d, pitch_rate_pid;
	double roll_pid_q[TIMED_MULTIPLEX_CONTROL_DELAY_COUNT], pitch_pid_q[TIMED_MULTIPLEX_CONTROL_DELAY_COUNT];
	double roll_pid_q_avg, pitch_pid_q_avg;
	double m_dt;
	double m_pwm;

	double gx_mean, gy_mean;
	double gz_lpf;

	double roll_setPoint;
	double pitch_setPoint;
	double yaw_setPoint;
	double roll_err, pitch_err, yaw_err;
	double roll_err_integral, pitch_err_integral, yaw_err_integral;
	double i_wind_up_limit, i_wind_up_limit_yaw;
	double p_gain_yaw, i_gain_yaw, d_gain_yaw;

	bool isFirst;

	double f_right, f_left, f_front, f_back;

	unsigned char* dataBuffer;

	double ctl_gain;

	double tmp_pid;

public:
	RolypolyController();
	~RolypolyController();

	void initialize();
	void destroy();

	void attitudeControl();
	void generatePWM(int* pwm1, int* dir1, int* pwm2, int* dir2);

	// 중첩 제어
	void AngularControl(double dt);
	void AngularRateControl();

	// getter, setter method
	void setDt(double dt) { m_dt = dt; }	// 필수
	void setEulerAngle(double roll, double pitch, double yaw);	// 필수
	void setSmoothValue(double xValue, double yValue, double zValue);	// 필수
	void setPwm(double pwm) { m_pwm = pwm; }	// 필수
	void setSetPoint(double roll, double pitch);	// 필수
	void setDataBuffer(unsigned char* buf) { dataBuffer = buf; }
	void setPidGain(double p, double i, double d) {
		p_gain = p; i_gain = i; d_gain = d;
	}
};