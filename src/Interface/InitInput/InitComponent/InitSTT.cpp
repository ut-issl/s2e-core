
#include "../Initialize.h"
#include "../../../Component/AOCS/STT.h"
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;


STT InitSTT(ClockGenerator* clock_gen, int sensor_id, const string fname, double step_time, const Dynamics *dynamics, const LocalEnvironment* local_env){

    IniAccess STT_conf(fname);
	const string st_sensor_id = std::to_string(static_cast<long long>(sensor_id));
	const char *cs = st_sensor_id.data();
  string section_tmp = "STT";
  section_tmp += cs;
	const char *STTSection = section_tmp.data();

	Quaternion q_b2c;
	STT_conf.ReadQuaternion(STTSection, "q_b2c",q_b2c);
	double sigma_ortho = STT_conf.ReadDouble(STTSection, "sigma_ortho");
	double sigma_sight = STT_conf.ReadDouble(STTSection, "sigma_sight");
	double output_delay_sec = STT_conf.ReadDouble(STTSection, "output_delay");//出力遅れ[sec]読み込み
	int output_delay = max(int(output_delay_sec / step_time),1);//intに変換
	double output_interval_sec = STT_conf.ReadDouble(STTSection, "output_interval");//出力間隔[sec]読み込み
	int output_interval = max(int(output_interval_sec / step_time),1);//intに変換
	double sun_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "sun_forbidden_angle");
	double sun_forbidden_angle_rad = sun_forbidden_angle_deg*M_PI / 180;  //deg→rad変換
	double earth_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "earth_forbidden_angle");
	double earth_forbidden_angle_rad = earth_forbidden_angle_deg*M_PI / 180;  //deg→rad変換
	double moon_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "moon_forbidden_angle");
	double moon_forbidden_angle_rad = moon_forbidden_angle_deg*M_PI / 180;  //deg→rad変換
	double capture_rate = STT_conf.ReadDouble(STTSection, "capture_rate");

	STT stt(clock_gen, q_b2c, sigma_ortho, sigma_sight, step_time, output_delay, output_interval,
	        sun_forbidden_angle_rad, earth_forbidden_angle_rad, moon_forbidden_angle_rad, capture_rate, dynamics, local_env);
	return stt;
}

