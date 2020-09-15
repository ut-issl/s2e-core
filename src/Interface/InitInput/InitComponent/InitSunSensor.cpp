#define _CRT_SECURE_NO_WARNINGS
#define PI   3.14159265358979323846
#include <string.h>
#include "../Initialize.h"
#include "../../../Component/AOCS/SunSensor.h"

//太陽センサ初期化, sensor_idで対応するセンサ読み込み
SunSensor InitSunSensor(ClockGenerator* clock_gen, int ss_id, string file_name, const SRPEnvironment* srp){

    IniAccess ss_conf(file_name);

	const string st_ss_id = std::to_string(static_cast<long long>(ss_id));
	const char *cs = st_ss_id.data();

	char Section[30] = "SUNSENSOR";
	strcat(Section, cs);

	Quaternion q_b2c;
	ss_conf.ReadQuaternion(Section, "q_b2c", q_b2c);

	double detectable_angle_deg = 0.0, detectable_angle_rad = 0.0;
	detectable_angle_deg = ss_conf.ReadDouble(Section, "detectable_angle_deg");
	detectable_angle_rad = PI/180.0*detectable_angle_deg;

	double ss_wnvar = 0.0;
	ss_wnvar = ss_conf.ReadDouble(Section, "ss_wnvar");

	double ss_bivar = 0.0;
	ss_bivar = ss_conf.ReadDouble(Section, "ss_bivar");

	SunSensor ss(clock_gen, q_b2c, detectable_angle_rad, ss_wnvar, ss_bivar, srp);
	return ss;
}