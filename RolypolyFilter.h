

#define MEAN_Q_LENGTH 3
#define ALT_Q_LENGTH 20

class RolypolyFilter {
	
public:

	double cf_kp, cf_ki;
	double m_ax, m_ay, m_az, m_gx, m_gy, m_gz;
	double m_roll, m_pitch, m_yaw;
	double m_roll_rad, m_pitch_rad;
	double cf_err_x, cf_err_sum_x, cf_err_y, cf_err_sum_y;
	double cf_kp_val_x, cf_ki_val_x, cf_kd_val_x, cf_kp_val_y, cf_ki_val_y, cf_kd_val_y;
	double cf_pid_out_x, cf_pid_out_y;
	double integral_gx, integral_gy;
	double compensated_by_pitch;
	double yaw_output;
	double m_dt;
	double m_gyroScaleFactor, m_accelScaleFactor;
	double mean_q_gx[MEAN_Q_LENGTH], mean_q_gy[MEAN_Q_LENGTH];
	double prevPhi, prevTheta, prevPsi;
	double phi, theta, psi;
	int alt[ALT_Q_LENGTH];
	double alt_sum, alt_avg;
	double roll_acc, pitch_acc, yaw_acc;
	double roll_gyro, pitch_gyro, yaw_gyro;
	double roll_gyro_dot, pitch_gyro_dot, yaw_gyro_dot;
	double isFirst;
	double zeroG_ax, zeroG_ay, zeroG_az;
	double vel_xa, vel_ya, vel_za;
	double vel_xg, vel_yg;
	double weight_body;
	//double pos_x, pos_y;
	//int m_gx1;
	//double integral_gx_trapez;

	double abs_vector;
	double scale_factor;


public:
	RolypolyFilter();
	~RolypolyFilter();

	void initialize(int ax, int ay, int az);
	void destroy();


	
	void complementaryFilter();
	void integralYaw();
	double generalLowPassFilter(double* lpf, double data, double a, double b);
	double generalMeanFilter(double* q, double len, double data);
	void GyroToEuler();
	double alt_filter(int ultra, int length);
	void accelerometerHSR();

	void estimateVelbyAccel();
	void estimateVelbyGyro();

	
	// getter, setter method
	void setRawData(int ax, int ay, int az, int gx, int gy, int gz);	// 필수
	void setDt(double dt) { m_dt = dt; }	// 필수
	void setGyroScaleFactor(double sf) { m_gyroScaleFactor = sf; }	// 필수
	void setAccelScaleFactor(double sf) { m_accelScaleFactor = sf; }	// 필수
	double getRoll() { return m_roll; }
	double getPitch() { return m_pitch; }
	double getYaw() { return yaw_gyro; }
	void setRoll(double roll) { m_roll = roll; }
	void setpitch(double pitch) { m_pitch = pitch; }
	void setyaw(double yaw) { m_yaw = yaw; }
	void Offset(double r, double p);

};