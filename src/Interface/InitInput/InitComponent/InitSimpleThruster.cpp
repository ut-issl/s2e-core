#include "../Initialize.h"
#include "../../../Component/Propulsion/SimpleThruster.h"

SimpleThruster InitSimpleThruster(int thruster_id, const std::string fname){
	IniAccess thruster_conf(fname);

    string sectionstr = "THRUSTER" + to_string(thruster_id);
    auto* Section = sectionstr.c_str();

	//スラスタ製品名
	char comp_name_char[100];
	thruster_conf.ReadChar(Section, "comp_name", 100, comp_name_char);
	string comp_name = comp_name_char;

	//スラスタ位置
	Vector<3> thruster_pos;
	thruster_conf.ReadVector(Section, "thruster_pos", thruster_pos);

	//スラスト方向ベクトル
	Vector<3> thruster_dir;
	thruster_conf.ReadVector(Section, "thruster_dir", thruster_dir);

    //スラスト大きさ誤差
    double max_mag = thruster_conf.ReadDouble(Section, "max_mag");

	//スラスト大きさ誤差
	double mag_err;
	mag_err = thruster_conf.ReadDouble(Section,"mag_err");

	//スラスト方向ベクトル誤差
	double deg_err;
	deg_err = thruster_conf.ReadDouble(Section,"deg_err");

	SimpleThruster thruster(thruster_pos, thruster_dir, max_mag, mag_err, deg_err, thruster_id);
	return thruster;
}
