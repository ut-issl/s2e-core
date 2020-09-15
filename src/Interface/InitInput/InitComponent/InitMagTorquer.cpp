#define _CRT_SECURE_NO_WARNINGS
#include <string.h>
#include "../Initialize.h"
#include "../../../Component/AOCS/MagTorquer.h"

//磁気トルカ初期化, actuator_idで対応するアクチュエータ読み込み
MagTorquer InitMagTorquer(ClockGenerator* clock_gen, int actuator_id, const string fname){

	IniAccess magtorquer_conf(fname);

	const string st_actuator_id = std::to_string(static_cast<long long>(actuator_id));
	const char *cs = st_actuator_id.data();

	char MTSection[30] = "MAGTORQUER";
	strcat(MTSection, cs);
	
	Quaternion q_b2c;
	magtorquer_conf.ReadQuaternion(MTSection, "q_b2c", q_b2c);

	Vector<3> max_c;
	magtorquer_conf.ReadVector(MTSection, "Max_c", max_c);

	Vector<3> min_c;
	magtorquer_conf.ReadVector(MTSection, "Min_c", min_c);

	Vector<3> bias_c;
	magtorquer_conf.ReadVector(MTSection, "Bias_c", bias_c);


	double rw_stepwidth;
	rw_stepwidth = magtorquer_conf.ReadDouble(MTSection, "rw_stepwidth");
	Vector<3> rw_stddev_c;
	magtorquer_conf.ReadVector(MTSection, "rw_stddev_c",rw_stddev_c);
	Vector<3> rw_limit_c;
	magtorquer_conf.ReadVector(MTSection, "rw_limit_c",rw_limit_c);
	Vector<3> nr_stddev_c;
	magtorquer_conf.ReadVector(MTSection, "nr_stddev_c",nr_stddev_c);

	int resolution;
	resolution = magtorquer_conf.ReadInt(MTSection, "resolution");


	MagTorquer magtorquer(clock_gen, actuator_id, q_b2c, max_c, min_c, bias_c, rw_stepwidth,
	        rw_stddev_c, rw_limit_c, nr_stddev_c, resolution);
	return magtorquer;	
	
}