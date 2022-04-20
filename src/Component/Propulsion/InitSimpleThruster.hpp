#pragma once

#include <Component/Propulsion/SimpleThruster.h>

SimpleThruster InitSimpleThruster(ClockGenerator* clock_gen, int thruster_id, const std::string fname, const Structure* structure,
                                  const Dynamics* dynamics);
SimpleThruster InitSimpleThruster(ClockGenerator* clock_gen, PowerPort* power_port, int thruster_id, const std::string fname,
                                  const Structure* structure, const Dynamics* dynamics);
