#ifndef __Initialize_H__
#define __Initialize_H__

#include <string>
#include <vector>

#include "IniAccess.h"
#define MAX_CHAR_NUM 256
#define CALC_LABEL "calculation"
#define LOG_LABEL "logging"

#include "../../Dynamics/Dynamics.h"
#include "../../Library/math/MatVec.hpp"
#include "../../Library/math/Matrix.hpp"
#include "../../Library/math/Quaternion.hpp"
#include "../../Library/math/Vector.hpp"
#include "../../Simulation/Spacecraft/Structure/Structure.h"
#include "../SpacecraftInOut/Ports/PowerPort.h"

using libra::Matrix;
using libra::Quaternion;
using libra::Vector;

// Logger
class Logger;
Logger *InitLog(std::string file_name);
Logger *InitLogMC(std::string file_name, bool enable);

// Structure
class KinematicsParams;
class Surface;
class RMMParams;
KinematicsParams InitKinematicsParams(std::string ini_path);
vector<Surface> InitSurfaces(std::string ini_path);
RMMParams InitRMMParams(std::string ini_path);

// Global Environment
class SimTime;
class ClockGenerator;
class CelestialInformation;
class HipparcosCatalogue;
class GnssSatellites;
SimTime *InitSimTime(std::string file_name);
CelestialInformation *InitCelesInfo(std::string file_name);
HipparcosCatalogue *InitHipCatalogue(std::string file_name);
GnssSatellites *InitGnssSatellites(std::string file_name);

// Local Environment
class MagEnvironment;
class SRPEnvironment;
class Atmosphere;
class LocalCelestialInformation;
class LocalEnvironment;
MagEnvironment InitMagEnvironment(std::string ini_path);
SRPEnvironment InitSRPEnvironment(std::string ini_path);
Atmosphere InitAtmosphere(std::string ini_path);

// Dynamics
class Dynamics;
class Attitude;
class Orbit;
class Temperature;
class Node;
class RelativeInformation;
Attitude *InitAttitude(std::string file_name, const Orbit *orbit,
                       const LocalCelestialInformation *celes_info,
                       const double step_sec, const Matrix<3, 3> inertia_tensor,
                       const int sat_id);
Orbit *
InitOrbit(const CelestialInformation *celes_info, std::string ini_path,
          double stepSec, double current_jd, double gravity_constant,
          std::string section = "ORBIT",
          RelativeInformation *rel_info = (RelativeInformation *)nullptr);
Temperature *InitTemperature(std::string ini_path,
                             const double rk_prop_step_sec);
Node InitNode(const vector<std::string> &nodestr);

// Disturbance
class RMMParams;
class AirDrag;
class SolarRadiation;
class GGDist;
class MagDisturbance;
class GeoPotential;
class ThirdBodyGravity;
AirDrag InitAirDrag(std::string ini_path, const vector<Surface> &surfaces,
                    const Vector<3> cg_b);
SolarRadiation InitSRDist(std::string ini_path, const vector<Surface> &surfaces,
                          Vector<3> cg_b);
GGDist InitGGDist(std::string ini_path);
MagDisturbance InitMagDisturbance(std::string ini_path, RMMParams rmm_params);
GeoPotential InitGeoPotential(std::string ini_path);
ThirdBodyGravity InitThirdBodyGravity(std::string ini_path,
                                      std::string ini_path_celes);

// Component
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
Gyro InitGyro(ClockGenerator *clock_gen, int sensor_id, const std::string fname,
              double compo_step_time, const Dynamics *dynamics);
Gyro InitGyro(ClockGenerator *clock_gen, PowerPort *power_port, int sensor_id,
              const std::string fname, double compo_step_time,
              const Dynamics *dynamics);
MagSensor InitMagSensor(ClockGenerator *clock_gen, int sensor_id,
                        const std::string fname, double compo_step_time,
                        const MagEnvironment *magnet);
MagSensor InitMagSensor(ClockGenerator *clock_gen, PowerPort *power_port,
                        int sensor_id, const std::string fname,
                        double compo_step_time, const MagEnvironment *magnet);
MagTorquer InitMagTorquer(ClockGenerator *clock_gen, int actuator_id,
                          const std::string fname, double compo_step_time,
                          const MagEnvironment *mag_env);
MagTorquer InitMagTorquer(ClockGenerator *clock_gen, PowerPort *power_port,
                          int actuator_id, const std::string fname,
                          double compo_step_time,
                          const MagEnvironment *mag_env);
RWModel InitRWModel(ClockGenerator *clock_gen, int actuator_id,
                    std::string file_name, double prop_step,
                    double compo_update_step);
RWModel InitRWModel(ClockGenerator *clock_gen, PowerPort *power_port,
                    int actuator_id, std::string file_name, double prop_step,
                    double compo_update_step);
STT InitSTT(ClockGenerator *clock_gen, int sensor_id, const std::string fname,
            double compo_step_time, const Dynamics *dynamics,
            const LocalEnvironment *local_env);
STT InitSTT(ClockGenerator *clock_gen, PowerPort *power_port, int sensor_id,
            const std::string fname, double compo_step_time,
            const Dynamics *dynamics, const LocalEnvironment *local_env);
SunSensor InitSunSensor(ClockGenerator *clock_gen, int sensor_id,
                        const std::string fname, const SRPEnvironment *srp);
SunSensor InitSunSensor(ClockGenerator *clock_gen, PowerPort *power_port,
                        int sensor_id, const std::string fname,
                        const SRPEnvironment *srp);
GNSSReceiver InitGNSSReceiver(ClockGenerator *clock_gen, int id,
                              const std::string fname, const Dynamics *dynamics,
                              const GnssSatellites *gnss_satellites,
                              const SimTime *simtime);
GNSSReceiver InitGNSSReceiver(ClockGenerator *clock_gen, PowerPort *power_port,
                              int id, const std::string fname,
                              const Dynamics *dynamics,
                              const GnssSatellites *gnss_satellites,
                              const SimTime *simtime);
SimpleThruster InitSimpleThruster(ClockGenerator *clock_gen, int thruster_id,
                                  const std::string fname,
                                  const Structure *structure,
                                  const Dynamics *dynamics);
SimpleThruster InitSimpleThruster(ClockGenerator *clock_gen,
                                  PowerPort *power_port, int thruster_id,
                                  const std::string fname,
                                  const Structure *structure,
                                  const Dynamics *dynamics);

BAT InitBAT(ClockGenerator *clock_gen, int bat_id, const std::string fname);
SAP InitSAP(ClockGenerator *clock_gen, int sap_id, const std::string fname,
            const SRPEnvironment *srp);
EMDS InitEMDS(int actuator_id);
UWBSensor InitUWBSensor(int sensor_id);
ANT InitANT(int ant_id, const std::string fname);
GScalculator InitGScalculator(const std::string fname);
Telescope InitTelescope(ClockGenerator *clock_gen, int sensor_id,
                        const std::string fname, const Attitude *attitude,
                        const HipparcosCatalogue *hipp,
                        const LocalCelestialInformation *local_celes_info);
// MCSim
class MCSimExecutor;
MCSimExecutor *InitMCSim(std::string file_name);
/*
// HILS
class HardwareMessage;
HardwareMessage* Init_HardwareMessage(std::string file_name);
class COSMOSWrapper;
COSMOSWrapper* Init_COSMOSWrapper(std::string file_name);
*/
#endif //__Initialize_H__
