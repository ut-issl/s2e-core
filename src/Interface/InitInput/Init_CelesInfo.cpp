#include <stdlib.h>
#include "../../Environment/CelestialInformation.h"
#include "Initialize.h"

CelestialInformation InitCelesInfo(string file_name)
{
	IniAccess ini_file(file_name);
	char* section = "PLANET_SELECTION";
  char* furnsh_section = "FURNSH_PATH";

	//各種定義読み込み
  string inertial_frame = ini_file.ReadString(section, "inertial_frame");
  string aber_cor = ini_file.ReadString(section, "aberration_correction");
  string center_obj = ini_file.ReadString(section, "center_object");

	//SPICE Furnsh
  vector<string> keywords = {"TLS", "TPC1", "TPC2", "TPC3", "BSP"};
  for(int i=0;i<keywords.size();i++)
  { 
    string fname = ini_file.ReadString(furnsh_section, keywords[i].c_str());
    furnsh_c(fname.c_str());
  }

	//天体情報をinitialize
	const int num_of_selected_body = ini_file.ReadInt(section, "num_of_selected_body");
	int* selected_body = new int[num_of_selected_body];	// これのdeleteはCelestialInformationに任せる
	SpiceInt planet_id; SpiceBoolean found;
	for (int i=0; i < num_of_selected_body; i++)
	{
		string selected_body_i = "selected_body(" + to_string(i) + ")";
		char selected_body_temp[30];
		ini_file.ReadChar(section, selected_body_i.c_str(), 30, selected_body_temp);
		bodn2c_c(selected_body_temp, (SpiceInt*)&planet_id, (SpiceBoolean*)&found);
		string body_name = selected_body_temp;
		// Error indication
		if(found == SPICEFALSE){
		    string message = "Input Celestial Object: '" + body_name + "' is not found.\nBe sure the name is defined in SPICE.\n";
        //TODO: manage MessageBox in windows
        //MessageBox(NULL, TEXT("One or More Input Celestial Object is not found.\nBe sure the name is defined in SPICE.\n"), TEXT("ERROR INDICATION"), MB_OK);
			exit(1);
		}
		selected_body[i] = planet_id;
	}
	CelestialInformation celestial_info(inertial_frame, aber_cor, center_obj, num_of_selected_body, selected_body);
	return celestial_info;
}
