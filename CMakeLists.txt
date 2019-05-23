cmake_minimum_required( VERSION 2.8 )
project( Robot-Control-OpenSim )

set( CMAKE_C_STANDARD 99 )
set( CMAKE_C_STANDARD_REQUIRED ON )
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

set( BUILD_LEGACY OFF CACHE BOOL "Build plug-in for OpenSim 3.x" )

set( OPENSIM_HOME ${CMAKE_SOURCE_DIR}/OpenSim CACHE PATH "OpenSim installation path" )
set( SIMBODY_HOME ${CMAKE_SOURCE_DIR}/simbody CACHE PATH "Simbody installation path" )

set( MODULES_DIR ${CONTROL_LIBRARY_DIR} CACHE PATH "Plug-in output directory" )

if( BUILD_LEGACY )
  find_package( Simbody 3.5 PATHS "${SIMBODY_HOME}" NO_MODULE NO_DEFAULT_PATH )
  include_directories( ${CMAKE_SOURCE_DIR} ${OPENSIM_HOME}/sdk/include/ ${OPENSIM_HOME}/sdk/include/OpenSim/ ${OPENSIM_HOME}/sdk/include/SimTK/simbody )
  set( OPENSIM_INSTALL_DIR "${OPENSIM_HOME}" CACHE PATH "Path to OpenSim Install." )
  set( CMAKE_MODULE_PATH "${OPENSIM_HOME}")
  find_package( OpenSim 3.3 MODULE )
  add_library( OpenSimModel MODULE open_sim_model-legacy.cpp data_io.cpp data_logging.cpp sensors.cpp signal_processing.cpp )
  target_compile_definitions( OpenSimModel PUBLIC -DOSIM_LEGACY )
else()
  find_package( Simbody 3.6 PATHS "${SIMBODY_HOME}" NO_MODULE NO_DEFAULT_PATH )
  find_package( OpenSim 4.0 PATHS "${OPENSIM_HOME}" NO_MODULE NO_DEFAULT_PATH )
  add_library( OpenSimModel MODULE open_sim_model.cpp data_io.cpp data_logging.cpp sensors.cpp signal_processing.cpp )
  target_include_directories( OpenSimModel PUBLIC ${CMAKE_SOURCE_DIR} ${OpenSim_INCLUDE_DIRS} ${Simbody_INCLUDE_DIRS} )
endif()

mark_as_advanced( Simbody_DIR )
mark_as_advanced( OpenSim_DIR )

set_target_properties( OpenSimModel PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${MODULES_DIR}/robot_control )
set_target_properties( OpenSimModel PROPERTIES PREFIX "" )
target_link_libraries( OpenSimModel ${OpenSim_LIBRARIES} ${Simbody_LIBRARIES} )

# install( TARGETS RobRehabGui RobRehabServer RUNTIME DESTINATION . )
