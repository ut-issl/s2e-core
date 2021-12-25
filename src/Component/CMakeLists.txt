cmake_minimum_required(VERSION 3.10)
project(COMPONENT)
add_library(${PROJECT_NAME} STATIC
  Abstract/ComponentBase.cpp
  Abstract/ComponentBase.h
  Abstract/EXP.cpp
  Abstract/EXP.h
  Abstract/IGPIOCompo.h
  Abstract/ITickable.h
  Abstract/Singleton.h
  Abstract/StateMachine.h
  AOCS/EMDS.cpp
  AOCS/EMDS.h
  AOCS/Gyro.cpp
  AOCS/Gyro.h
  AOCS/MagSensor.cpp
  AOCS/MagSensor.h
  AOCS/MagTorquer.cpp
  AOCS/MagTorquer.h
  AOCS/rw_ode.cpp
  AOCS/rw_ode.hpp
  AOCS/RWModel.cpp
  AOCS/RWModel.h
#  AOCS/RWModule.cpp
#  AOCS/RWModule.h
  AOCS/STT.cpp
  AOCS/STT.h
  AOCS/SunSensor.cpp
  AOCS/SunSensor.h
  AOCS/UWBSensor.cpp
  AOCS/UWBSensor.h
  AOCS/GNSSReceiver.cpp
  CommGS/ANT.cpp
  CommGS/GScalculator.cpp
  CDH/OBC.cpp
  CDH/OBC.h
  Logic/LPF.cpp
  Logic/LPF.h
  Logic/PIDController.h
  Logic/RVDController.cpp
  Logic/RVDController.h
  Logic/UWBEstimator.cpp
  Logic/UWBEstimator.h
  Power/BAT.cpp
  Power/BAT.h
  Power/PCU_InitialStudy.cpp
  Power/PCU_InitialStudy.h
  Mission/Telescope/Telescope.cpp
  Mission/Telescope/Telescope.h
  Power/SAP.cpp
  Power/SAP.h
  Propulsion/SimpleThruster.cpp
  Propulsion/SimpleThruster.h
    
## C++/CLI related codes
#  CDH/TMTCInterface.cpp
#  CDH/TMTCInterface.h

)
# Compile option
if(MSVC)
    target_compile_options(${PROJECT_NAME} PUBLIC "/W4")
	target_compile_options(${PROJECT_NAME} PUBLIC "/source-charset:utf-8")
else()
    target_compile_options(${PROJECT_NAME} PUBLIC "-Wall")
    set(CMAKE_CXX_FLAGS "-m32 -rdynamic -Wall -g")
    set(CMAKE_C_FLAGS "-m32 -rdynamic -Wall -g")
endif()
