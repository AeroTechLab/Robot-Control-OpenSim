cmake_minimum_required( VERSION 2.8 )
project( Robot-Control-OpenSim )

set( CMAKE_C_STANDARD 99 )
set( CMAKE_C_STANDARD_REQUIRED ON )
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

set( BUILD_LEGACY OFF CACHE BOOL "Build plug-in for OpenSim 3.x" FORCE )

set( OPENSIM_DIR ${CMAKE_SOURCE_DIR}/OpenSim CACHE PATH "OpenSim installation directory" FORCE )
set( SIMBODY_DIR ${CMAKE_SOURCE_DIR}/simbody CACHE PATH "Simbody installation directory" FORCE )
set( CONTROL_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/../Robot-Control-Library CACHE PATH "Robot Control Library base directory" FORCE )
set( MODULES_DIR ${CONTROL_LIBRARY_DIR}/robot_control CACHE PATH "Plug-in output directory" FORCE )

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${MODULES_DIR} )

#include_directories( ${CONTROL_LIBRARY_DIR} ${OPENSIM_DIR}/include ${SIMBODY_DIR}/include )
link_directories( ${OPENSIM_DIR}/lib ${SIMBODY_DIR}/lib )

if( BUILD_LEGACY )
  add_library( OpenSimModel MODULE open_sim_model-legacy.cpp )
else()
  add_library( OpenSimModel MODULE open_sim_model.cpp )
endif()

set_target_properties( OpenSimModel PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/robot_control/ )
set_target_properties( OpenSimModel PROPERTIES PREFIX "" )
target_include_directories( OpenSimModel PUBLIC ${CONTROL_LIBRARY_DIR} ${OPENSIM_DIR}/include ${SIMBODY_DIR}/include )
target_link_libraries( OpenSimModel -lSimTKcommon -lSimTKmath -lSimTKsimbody -losimActuators -losimAnalyses -losimCommon -losimLepton -losimSimulation -losimTools )

# install( TARGETS RobRehabGui RobRehabServer RUNTIME DESTINATION . )
