#define _CRT_SECURE_NO_WARNINGS
#include <string.h>
#include "../Initialize.h"
#include "../../../Component/AOCS/MagSensor.h"

//磁気センサ初期化, sensor_idで対応するセンサ読み込み
MagSensor InitMagSensor(ClockGenerator* clock_gen, int sensor_id, const string fname, const MagEnvironment* magnet){

	IniAccess magsensor_conf(fname);

	const string st_sensor_id = std::to_string(static_cast<long long>(sensor_id));
	const char *cs = st_sensor_id.data();

	char MSSection[30] = "MAGSENSOR";
	strcat(MSSection, cs);
	
	Quaternion q_b2c;
	magsensor_conf.ReadQuaternion(MSSection, "q_b2c", q_b2c);

	Vector<9> sf_vec;
	magsensor_conf.ReadVector(MSSection, "ScaleFactor", sf_vec);
	Matrix<3,3> scale_factor;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
		scale_factor[i][j] = sf_vec[i * 3 + j];
		}
	}


	Vector<3> bias_c;
	magsensor_conf.ReadVector(MSSection,"Bias_c",bias_c);

	double rw_stepwidth;
	rw_stepwidth = magsensor_conf.ReadDouble(MSSection, "rw_stepwidth");
	Vector<3> rw_stddev_c;
	magsensor_conf.ReadVector(MSSection, "rw_stddev_c",rw_stddev_c);
	Vector<3> rw_limit_c;
	magsensor_conf.ReadVector(MSSection, "rw_limit_c",rw_limit_c);
	Vector<3> nr_stddev_c;
	magsensor_conf.ReadVector(MSSection, "nr_stddev_c",nr_stddev_c);

	MagSensor magsensor(clock_gen, sensor_id, q_b2c, scale_factor, bias_c, rw_stepwidth, rw_stddev_c, rw_limit_c, nr_stddev_c, magnet);
	return magsensor;	
	
}