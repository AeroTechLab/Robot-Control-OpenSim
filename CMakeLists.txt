cmake_minimum_required( VERSION 2.8 )
project( Robot-Control-OpenSim )

set( CMAKE_C_STANDARD 99 )
set( CMAKE_C_STANDARD_REQUIRED ON )
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

set( BUILD_LEGACY OFF CACHE BOOL "Build plug-in for OpenSim 3.x" )

set( OPENSIM_HOME ${CMAKE_SOURCE_DIR}/OpenSim CACHE PATH "OpenSim installation path" )
set( SIMBODY_HOME ${CMAKE_SOURCE_DIR}/simbody CACHE PATH "Simbody installation path" )
set( CONTROL_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/../Robot-Control-Library CACHE PATH "Robot Control Library base directory" )
set( UTILS_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/../Platform-Utils CACHE PATH "Platform Utils library base directory" )
set( MODULES_DIR ${CONTROL_LIBRARY_DIR} CACHE PATH "Plug-in output directory" )

if( BUILD_LEGACY )
  find_package( Simbody 3.5 PATHS "${SIMBODY_HOME}" NO_MODULE NO_DEFAULT_PATH )
  find_package( OpenSim 3.3 PATHS "${OPENSIM_HOME}" NO_MODULE NO_DEFAULT_PATH )
  add_library( OpenSimModel MODULE open_sim_model-legacy.cpp )
else()
  find_package( Simbody 3.6 PATHS "${SIMBODY_HOME}" NO_MODULE NO_DEFAULT_PATH )
  find_package( OpenSim 4.0 PATHS "${OPENSIM_HOME}" NO_MODULE NO_DEFAULT_PATH )
  add_library( OpenSimModel MODULE open_sim_model.cpp )
endif()

mark_as_advanced( Simbody_DIR )
mark_as_advanced( OpenSim_DIR )

set_target_properties( OpenSimModel PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${MODULES_DIR}/robot_control )
set_target_properties( OpenSimModel PROPERTIES PREFIX "" )
target_include_directories( OpenSimModel PUBLIC ${CONTROL_LIBRARY_DIR} ${UTILS_LIBRARY_DIR} ${OpenSim_INCLUDE_DIRS} ${Simbody_INCLUDE_DIRS} )
target_link_libraries( OpenSimModel ${OpenSim_LIBRARIES} ${Simbody_LIBRARIES} )

# install( TARGETS RobRehabGui RobRehabServer RUNTIME DESTINATION . )
