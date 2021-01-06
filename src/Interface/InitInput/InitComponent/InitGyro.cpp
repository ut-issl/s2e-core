#define _CRT_SECURE_NO_WARNINGS
#include "../Initialize.h"
#include <string.h>
#include "../../../Component/AOCS/Gyro.h"

// ジャイロ初期化, sensor_idで対応するセンサ読み込み
Gyro InitGyro(ClockGenerator* clock_gen, PowerPort* power_port, int sensor_id, int port_id, const string fname, const Dynamics* dynamics){

	IniAccess gyro_conf(fname);

	char GSection[30] = "GYRO";

	Quaternion q_b2c;
	gyro_conf.ReadQuaternion(GSection, "q_b2c", q_b2c);

	Vector<9> sf_vec;
	gyro_conf.ReadVector(GSection, "ScaleFactor", sf_vec);
	Matrix<3, 3> scale_factor;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			scale_factor[i][j] = sf_vec[i * 3 + j];
		}
	}

	Vector<3> bias_c;
	gyro_conf.ReadVector(GSection, "Bias_c", bias_c);

	double rw_stepwidth;
	rw_stepwidth = gyro_conf.ReadDouble(GSection, "rw_stepwidth");
	Vector<3> rw_stddev_c;
	gyro_conf.ReadVector(GSection, "rw_stddev_c", rw_stddev_c);
	Vector<3> rw_limit_c;
	gyro_conf.ReadVector(GSection, "rw_limit_c", rw_limit_c);
	Vector<3> nr_stddev_c;
	gyro_conf.ReadVector(GSection, "nr_stddev_c", nr_stddev_c);

	double range_to_const;
	range_to_const = gyro_conf.ReadDouble(GSection, "Range_to_const");
	double range_to_zero;
	range_to_zero = gyro_conf.ReadDouble(GSection, "Range_to_zero");

  double current;
  current = gyro_conf.ReadDouble(GSection, "current");

	Gyro gyro(clock_gen, power_port, sensor_id, port_id, q_b2c, scale_factor, bias_c, 
            rw_stepwidth, rw_stddev_c, rw_limit_c, nr_stddev_c, 
            range_to_const, range_to_zero, current, dynamics);

	return gyro;
}

Gyro InitGyro(ClockGenerator* clock_gen, int sensor_id, int port_id, const string fname, const Dynamics* dynamics){

	IniAccess gyro_conf(fname);

	char GSection[30] = "GYRO";

	Quaternion q_b2c;
	gyro_conf.ReadQuaternion(GSection, "q_b2c", q_b2c);

	Vector<9> sf_vec;
	gyro_conf.ReadVector(GSection, "ScaleFactor", sf_vec);
	Matrix<3, 3> scale_factor;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			scale_factor[i][j] = sf_vec[i * 3 + j];
		}
	}

	Vector<3> bias_c;
	gyro_conf.ReadVector(GSection, "Bias_c", bias_c);

	double rw_stepwidth;
	rw_stepwidth = gyro_conf.ReadDouble(GSection, "rw_stepwidth");
	Vector<3> rw_stddev_c;
	gyro_conf.ReadVector(GSection, "rw_stddev_c", rw_stddev_c);
	Vector<3> rw_limit_c;
	gyro_conf.ReadVector(GSection, "rw_limit_c", rw_limit_c);
	Vector<3> nr_stddev_c;
	gyro_conf.ReadVector(GSection, "nr_stddev_c", nr_stddev_c);

	double range_to_const;
	range_to_const = gyro_conf.ReadDouble(GSection, "Range_to_const");
	double range_to_zero;
	range_to_zero = gyro_conf.ReadDouble(GSection, "Range_to_zero");

  double current;
  current = gyro_conf.ReadDouble(GSection, "current");

	Gyro gyro(clock_gen, sensor_id, port_id, q_b2c, scale_factor, bias_c, 
            rw_stepwidth, rw_stddev_c, rw_limit_c, nr_stddev_c, 
            range_to_const, range_to_zero, current, dynamics);

	return gyro;
}