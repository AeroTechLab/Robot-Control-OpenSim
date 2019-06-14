cmake_minimum_required( VERSION 2.8 )
project( Robot-Control-OpenSim )

set( CMAKE_C_STANDARD 99 )
set( CMAKE_C_STANDARD_REQUIRED ON )
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

set( BUILD_LEGACY OFF CACHE BOOL "Build plug-in for OpenSim 3.x" )

add_library( OpenSimModel MODULE osim_model.cpp nms_processor-base.cpp nms_processor-osim.cpp )
add_library( OpenSimModelNN MODULE osim_model.cpp nms_processor-base.cpp nms_processor-nn.cpp )
add_library( OpenSimModelIK MODULE osim_model-ik.cpp nms_processor-base.cpp nms_processor-osim.cpp )
add_library( OpenSimModelIKNN MODULE osim_model-ik.cpp nms_processor-base.cpp nms_processor-nn.cpp )
add_executable( OpenSimModelBuilder osim_model_generator.cpp )
add_executable( OpenSimModelLoader osim_model_loader.cpp )

if( BUILD_LEGACY )
  find_package( Simbody 3.5 REQUIRED PATHS "${SIMBODY_HOME}" NO_MODULE NO_DEFAULT_PATH )
  include_directories( ${CMAKE_SOURCE_DIR} ${OPENSIM_HOME}/sdk/include/ ${OPENSIM_HOME}/sdk/include/OpenSim/ ${OPENSIM_HOME}/sdk/include/SimTK/simbody )
  set( OPENSIM_INSTALL_DIR "${OPENSIM_HOME}" CACHE PATH "Path to OpenSim Install." )
  set( CMAKE_MODULE_PATH "${OPENSIM_HOME}")
  find_package( OpenSim 3.3 REQUIRED MODULE )
  target_compile_definitions( OpenSimModel PUBLIC -DOSIM_LEGACY )
  target_compile_definitions( OpenSimModelNN PUBLIC -DOSIM_LEGACY -DUSE_NN )
  target_compile_definitions( OpenSimModelIK PUBLIC -DOSIM_LEGACY )
  target_compile_definitions( OpenSimModelIKNN PUBLIC -DOSIM_LEGACY -DUSE_NN )
  target_compile_definitions( OpenSimModelBuilder PUBLIC -DOSIM_LEGACY )
  target_compile_definitions( OpenSimModelLoader PUBLIC -DOSIM_LEGACY )
else()
  # Find the OpenSim libraries and header files.
  set( OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME} CACHE PATH "Top-level directory of OpenSim install." )
  # This command searches for the file OpenSimConfig.cmake
  # in common system directories and in OPENSIM_INSTALL_DIR.
  find_package( OpenSim 4.0 REQUIRED PATHS "${OPENSIM_INSTALL_DIR}" )
  include_directories( ${CMAKE_SOURCE_DIR} ${OpenSim_INCLUDE_DIRS} ${Simbody_INCLUDE_DIRS} )
  target_compile_definitions( OpenSimModelNN PUBLIC -DUSE_NN )
  target_compile_definitions( OpenSimModelIKNN PUBLIC -DUSE_NN )
endif()

mark_as_advanced( Simbody_DIR )
mark_as_advanced( OpenSim_DIR )

set_target_properties( OpenSimModel PROPERTIES LIBRARY_OUTPUT_DIRECTORY plugins/robot_control )
set_target_properties( OpenSimModel PROPERTIES PREFIX "" )
target_link_libraries( OpenSimModel ${OpenSim_LIBRARIES} ${Simbody_LIBRARIES} )

set_target_properties( OpenSimModelNN PROPERTIES LIBRARY_OUTPUT_DIRECTORY plugins/robot_control )
set_target_properties( OpenSimModelNN PROPERTIES PREFIX "" )
target_link_libraries( OpenSimModelNN ${OpenSim_LIBRARIES} ${Simbody_LIBRARIES} )

set_target_properties( OpenSimModelIK PROPERTIES LIBRARY_OUTPUT_DIRECTORY plugins/robot_control )
set_target_properties( OpenSimModelIK PROPERTIES PREFIX "" )
target_link_libraries( OpenSimModelIK ${OpenSim_LIBRARIES} ${Simbody_LIBRARIES} )

set_target_properties( OpenSimModelIKNN PROPERTIES LIBRARY_OUTPUT_DIRECTORY plugins/robot_control )
set_target_properties( OpenSimModelIKNN PROPERTIES PREFIX "" )
target_link_libraries( OpenSimModelIKNN ${OpenSim_LIBRARIES} ${Simbody_LIBRARIES} )

target_link_libraries( OpenSimModelBuilder ${OpenSim_LIBRARIES} ${Simbody_LIBRARIES} )
target_link_libraries( OpenSimModelLoader ${OpenSim_LIBRARIES} ${Simbody_LIBRARIES} )
