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

//Logger
class Logger;
Logger* InitLog(string file_name);
Logger* InitLogMC(string file_name, bool enable);
//Global Environment
class SimTime;
class CelestialInformation;
class HipparcosCatalogue;
SimTime* InitSimTime(string file_name);
CelestialInformation* InitCelesInfo(string file_name);
HipparcosCatalogue* InitHipCatalogue(string file_name);

//Local Environment
class MagEnvironment;
class SRPEnvironment;
class Atmosphere;
class LocalCelestialInformation;
class LocalEnvironment;
MagEnvironment InitMagEnvironment(string ini_path);
SRPEnvironment InitSRPEnvironment(string ini_path);
Atmosphere InitAtmosphere(string ini_path);

//Dynamics
class Dynamics;
class Attitude;
class Orbit;
class Temperature;
class Node;
Attitude* InitAttitude(string file_name, const Orbit* orbit, const LocalCelestialInformation* celes_info);
Orbit* InitOrbit(string ini_path, double stepSec, double current_jd, double gravity_constant, string section = "ORBIT");
Temperature* InitTemperature(IniAccess mainIni);
Node InitNode(const vector<string>& nodestr);

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
SAP InitSAP(int sap_id, const string fname, const SRPEnvironment* srp);
EMDS InitEMDS(int actuator_id);
UWBSensor InitUWBSensor(int sensor_id);
ANT InitANT(int ant_id, const string fname);
GScalculator InitGScalculator(const string fname);
Telescope InitTelescope(int sensor_id, const string fname, const Attitude * attitude, const HipparcosCatalogue* hipp, const LocalCelestialInformation *local_celes_info);
STT InitSTT(int sensor_id, const string fname, double step_time, const Dynamics *dynamics, const LocalEnvironment* local_env);
SunSensor InitSunSensor(int sensor_id, const string fname, const SRPEnvironment* srp);
GNSSReceiver InitGNSSReceiver(int id, const string fname, const Dynamics* dynamics);
//MCSim
class MCSimExecutor;
MCSimExecutor* InitMCSim(string file_name);
/*
// HILS
class HardwareMessage;
HardwareMessage* Init_HardwareMessage(string file_name);
class COSMOSWrapper;
COSMOSWrapper* Init_COSMOSWrapper(string file_name);
*/
#endif //__Initialize_H__
