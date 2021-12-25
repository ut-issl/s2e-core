#define _USE_MATH_DEFINES
#include "HipparcosCatalogue.h"
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>

using namespace std;

HipparcosCatalogue::HipparcosCatalogue(double max_magnitude, string catalogue_path) : max_magnitude_(max_magnitude), catalogue_path_(catalogue_path)
{
	
}

HipparcosCatalogue::~HipparcosCatalogue()
{

}

bool HipparcosCatalogue::ReadContents(const string& filename, const char delimiter = ',')
{	
	if (!IsCalcEnabled) return false;

	ifstream ifs(filename);
	if (!ifs.is_open())
	{
		cerr << "file open error(hip_main.csv)";
		return false;
	}

	string title; ifs >> title;//タイトルを読み飛ばす
	while (!ifs.eof())
	{
		HipData hipdata;

		string line;
		ifs >> line;
		replace(line.begin(), line.end(), delimiter, ' ');//stringstreamで自動的に数値を抽出するために、スペース区切りに直す
		istringstream streamline(line);

		streamline >> hipdata.hip_num >> hipdata.vmag >> hipdata.ra >> hipdata.de;

		if (hipdata.vmag > max_magnitude_) { return true; }//max_magnitudeで指定したvmagより暗い星は読み込まない
		hip_catalogue.push_back(hipdata);
	}

	return true;
}

libra::Vector<3> HipparcosCatalogue::GetStarDir_i(int rank) const
{
	libra::Vector<3> position;
	double ra = GetRA(rank) * M_PI / 180;
	double de = GetDE(rank) * M_PI / 180;

	position[0] = cos(ra) * cos(de);
	position[1] = sin(ra) * cos(de);
	position[2] = sin(de);

	return position;
}

libra::Vector<3> HipparcosCatalogue::GetStarDir_b(int rank, Quaternion q_i2b) const
{
	libra::Vector<3> position_i;
	libra::Vector<3> position_b;

	position_i = GetStarDir_i(rank);
	position_b = q_i2b.frame_conv(position_i);

	return position_b;
}

string HipparcosCatalogue::GetLogHeader() const
{
	string str_tmp = "";

	return str_tmp;
}

string HipparcosCatalogue::GetLogValue() const
{
	string str_tmp = "";

	return str_tmp;
}
