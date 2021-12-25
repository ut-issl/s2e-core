#pragma once

#include "./SimulationCase.h"

class Envir;
class Disturbances;
class SampleSat;
class SimTime;
class Logger;

class SampleCase : public SimulationCase
{
public:
  SampleCase(string ini_fname, string data_path);
  virtual ~SampleCase();
  void Initialize();
  void Main();

  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

private:
  string ini_fname;
  string data_path;
  Envir* environment;
  Disturbances* disturbances;
  SampleSat* sample_sat;
  SimTime* sim_time;
  Logger* default_log;
};
