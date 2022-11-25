#pragma once

#include <Component/Power/SAP.h>

SAP InitSAP(ClockGenerator* clock_gen, int sap_id, const std::string fname, const SRPEnvironment* srp,
            const LocalCelestialInformation* local_celes_info, double compo_step_time);
SAP InitSAP(ClockGenerator* clock_gen, int sap_id, const std::string fname, const SRPEnvironment* srp, double compo_step_time);
