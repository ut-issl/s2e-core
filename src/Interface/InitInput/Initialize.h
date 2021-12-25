#ifndef __Initialize_H__
#define __Initialize_H__

#include "IniAccess.h"
#define MAX_CHAR_NUM 256
#define CALC_LABEL "calculation"
#define LOG_LABEL "logging"

#include "../../Library/math/Matrix.hpp"
#include "../../Library/math/Vector.hpp"
#include "../../Library/math/MatVec.hpp"
#include "../../Library/math/Quaternion.hpp"
#include "../../Dynamics/Dynamics.h"

#include <vector>
using libra::Matrix;
using libra::Vector;
using libra::Quaternion;

//SimBase
class Attitude;
class SimTime;
class Orbit;
Attitude* InitAttitude(string file_name, const Orbit* orbit, const CelestialInformation* celes_info);
SimTime* InitSimTime(string file_name);
Orbit* InitOrbit(string ini_path, double stepSec, double current_jd, string section = "ORBIT");
Logger* InitLog(string file_name);
Logger* InitLogMC(string file_name, bool enable);

//MCSim
class MCSimExecutor;
MCSimExecutor* InitMCSim(string file_name);

// HILS
class HardwareMessage;
HardwareMessage* Init_HardwareMessage(string file_name);
class COSMOSWrapper;
COSMOSWrapper* Init_COSMOSWrapper(string file_name);

//Celestial Information
class CelestialInformation;
CelestialInformation InitCelesInfo(string file_name);

//Hipparcos Catalogue
class HipparcosCatalogue;
HipparcosCatalogue InitHipCatalogue(string file_name);

//Environment
class MagEnvironment;
class SRPEnvironment;
class Atmosphere;

MagEnvironment InitMagEnvironment(string ini_path);
SRPEnvironment InitSRPEnvironment(string ini_path);
Atmosphere InitAtmosphere(string ini_path);

//Disturbance
class AirDrag;
class SolarRadiation;
class GGDist;
class MagDisturbance;
class GeoPotential;
class ThirdBodyGravity;

AirDrag InitAirDrag(string ini_path);
SolarRadiation InitSRDist(string ini_path);
GGDist InitGGDist(string ini_path);
MagDisturbance InitMagDisturbance(string ini_path);
GeoPotential InitGeoPotential(string ini_path);
ThirdBodyGravity InitThirdBodyGravity(string ini_path, string ini_path_celes);

// Dynamics
class Dynamics;
class Temperature;
class Node;

Temperature* InitTemperature(IniAccess mainIni);
Node InitNode(const vector<string>& nodestr);


//Component
class Gyro;
class MagSensor;
class MagTorquer;
class RWModel;
class SunSensor;
class SimpleThruster;
class BAT;
class SAP;
class EMDS;
class UWBSensor;
class ANT;
class GScalculator;
class Telescope;
class STT;
class GNSSReceiver;

Gyro InitGyro(int sensor_id, int port_id, const string fname, const Dynamics* dynamics);
MagSensor InitMagSensor(int sensor_id, const string fname, const MagEnvironment* magnet);
MagTorquer InitMagTorquer(int actuator_id, const string fname);
RWModel InitRWModel(int actuator_id, string file_name, double prop_step);
SimpleThruster InitSimpleThruster(int thruster_id);
BAT InitBAT(int bat_id, const string fname);
SAP InitSAP(int sap_id, const string fname, const SRPEnvironment* srp, const CelestialInformation* celestial);
EMDS InitEMDS(int actuator_id);
UWBSensor InitUWBSensor(int sensor_id);
ANT InitANT(int ant_id, const string fname);
GScalculator InitGScalculator(const string fname);
Telescope InitTelescope(int sensor_id, const string fname, const Dynamics* dynamics);
STT InitSTT(int sensor_id, const string fname, double step_time, const Dynamics *dynamics);
SunSensor InitSunSensor(int sensor_id, const string fname, const Dynamics *dynamics);
GNSSReceiver InitGNSSReceiver(int id, const string fname, const Dynamics* dynamics);

#endif //__Initialize_H__
