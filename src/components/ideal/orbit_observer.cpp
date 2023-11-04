/*
 * @file orbit_observer.cpp
 * @brief Ideal component which can observe orbit
 */

#include "orbit_observer.hpp"

OrbitObserver(const int prescaler, ClockGenerator* clock_generator, const Orbit* orbit) : Component(prescaler, clock_generator), orbit_(orbit) {}
