#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>

#include <iostream>
#include <string>
//#include <vector>

#include "interface/robot_control.h"

#ifndef USE_NN
  #include "emg_optimizer-nn.h"
#else
  #include "emg_optimizer-osim.h"
#endif

struct
{
  OpenSim::Model* osimModel;
  SimTK::State stateFD, stateIK;
  std::vector<OpenSim::CoordinateActuator*> actuatorsList;
  std::vector<int> accelerationIndexesList;
  SimTK::Array_<OpenSim::CoordinateReference> coordinateReferences;
  OpenSim::MarkerSet markers;
  OpenSim::MarkersReference markersReference;
  OpenSim::Set<OpenSim::MarkerWeight> markerWeights;
  std::vector<std::string> markerLabels;
  std::vector<SimTK::Vec3> markerSetpoints;
  std::vector<SimTK::Vec3> markerInitialLocations;
  SimTK::Matrix_<SimTK::Vec3> markerSetpointsTable;
  std::vector<char*> jointNamesList;
  std::vector<char*> axisNamesList;
  enum RobotState currentControlState;
  SimTK::Vector emgInputs;
  EMGOptimizerImpl* emgProcessor;
}
controller;


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


bool InitController( const char* data )
{ 
  const char* REFERENCE_AXIS_NAMES[ 3 ] = { "_x", "_y", "_z" };
  
  try 
  { // Create an OpenSim model from XML (.osim) file
    controller.osimModel = new OpenSim::Model( std::string( "config/robots/" ) + data + ".osim" );
    controller.osimModel->printBasicInfo( std::cout );
    controller.osimModel->setGravity( SimTK::Vec3( 0.0, -9.80665, 0.0 ) );

    const OpenSim::MarkerSet& markerSet = controller.osimModel->getMarkerSet(); std::cout << "OSim: found " << markerSet.getSize() << " markers" << std::endl;
    for( int markerIndex = 0; markerIndex < markerSet.getSize(); markerIndex++ )
    {
      std::string markerName = markerSet[ markerIndex ].getName();
      if( markerName.find( "_ref" ) != std::string::npos )
      {
        std::cout << "OSim: found reference marker " << markerName << std::endl;
        controller.markerWeights.adoptAndAppend( new OpenSim::MarkerWeight( markerName, 1.0 ) );
        controller.markers.adoptAndAppend( &(markerSet[ markerIndex ]) );
        controller.markerLabels.push_back( markerName );
        for( size_t referenceAxisIndex = 0; referenceAxisIndex < 3; referenceAxisIndex++ )
        {
          std::string markerAxisName = markerName + REFERENCE_AXIS_NAMES[ referenceAxisIndex ];
          controller.axisNamesList.push_back( (char*) markerAxisName.c_str() );
        }
      }
    }
    std::cout << "OSim: generated markers reference size" << controller.markersReference.updMarkerWeightSet().getSize() << std::endl;
    std::cout << "OSim: generated coordinates reference size: " << controller.coordinateReferences.size() << std::endl;
    const OpenSim::Set<OpenSim::Muscle>& muscleSet = controller.osimModel->getMuscles();
    const OpenSim::Set<OpenSim::Actuator>& actuatorSet = controller.osimModel->getActuators();
    controller.osimModel->buildSystem();
    for( int actuatorIndex = 0; actuatorIndex < actuatorSet.getSize(); actuatorIndex++ )
    {
      std::string actuatorName = actuatorSet[ actuatorIndex ].getName();
      if( not muscleSet.contains( actuatorName ) )
      {
        OpenSim::CoordinateActuator* actuator = dynamic_cast<OpenSim::CoordinateActuator*>(&(actuatorSet[ actuatorIndex ]));
        if( actuator != NULL )
        {
          std::cout << "Found coordinate " << actuator->getCoordinate()->getName() << " actuator " << actuator->getName() << std::endl;
//           controller.coordinateReferences.push_back( OpenSim::CoordinateReference( actuatorCoordinate.getName(), OpenSim::Constant( 0.0 ) ) );
          controller.actuatorsList.push_back( actuator );
          controller.jointNamesList.push_back( (char*) actuator->getCoordinate()->getName().c_str() );
          controller.accelerationIndexesList.push_back( controller.osimModel->getCoordinateSet().getIndex( actuator->getCoordinate() ) );
        }
      }
    }
    std::cout << "OpenSim model loaded successfully ! (" << controller.osimModel->getNumCoordinates() << " coordinates)" << std::endl;
    SimTK::State& stateFD = controller.osimModel->initializeState();    // Initialize the system
    for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
#ifdef OSIM_LEGACY
      muscleSet[ muscleIndex ].setDisabled( stateFD, true );
#else
      muscleSet[ muscleIndex ].setAppliesForce( stateFD, false );
#endif
    for( size_t jointIndex = 0; jointIndex < controller.actuatorsList.size(); jointIndex++ )
    {
#ifdef OSIM_LEGACY
      controller.actuatorsList[ jointIndex ]->overrideForce( stateFD, true );
#else
      controller.actuatorsList[ jointIndex ]->overrideActuation( stateFD, true );
#endif
      controller.actuatorsList[ jointIndex ]->getCoordinate()->setValue( stateFD, 0.0 );
    }
    
    controller.markerSetpoints.resize( controller.markers.getSize(), SimTK::Vec3( 0.0 ) );
    controller.markerInitialLocations.resize( controller.markers.getSize(), SimTK::Vec3( 0.0 ) );
    controller.markerSetpointsTable.resize( 1, controller.markers.getSize() );
    for( int markerIndex = 0; markerIndex < controller.markers.getSize(); markerIndex++ )
#ifdef OSIM_LEGACY
      controller.osimModel->getSimbodyEngine().getPosition( controller.stateFD, controller.markers[ markerIndex ].getBody(), controller.markers[ markerIndex ].getOffset(), controller.markerInitialLocations[ markerIndex ] );
#else
      controller.markerInitialLocations[ markerIndex ] = controller.markers[ markerIndex ].getLocationInGround( stateFD );
#endif
    
    controller.stateFD.setTime( 0.0 );
    controller.stateIK = controller.stateFD;
    
    controller.emgProcessor = new EMGOptimizerImpl( *(controller.osimModel), controller.actuatorsList, 1000 );
    
    SetControlState( /*ROBOT_PASSIVE*/ROBOT_PREPROCESSING );
  }
  catch( OpenSim::Exception ex )
  {
    std::cout << ex.getMessage() << std::endl;
    EndController();
    return false;
  }
  catch( std::exception ex )
  {
    std::cout << ex.what() << std::endl;
    EndController();
    return false;
  }
  catch( ... )
  {
    std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
    EndController();
    return false;
  }
  
  std::cout << "OpenSim model initialized successfully ! (" << controller.osimModel->getNumCoordinates() << " coordinates)" << std::endl;
  
  return true;
}

void EndController()
{
  controller.markers.clearAndDestroy();
  
  delete controller.osimModel;
  
  controller.jointNamesList.clear();
  controller.axisNamesList.clear();
  
  controller.emgInputs.clear();
}

size_t GetJointsNumber() { return (size_t) controller.jointNamesList.size(); }

const char** GetJointNamesList() { return (const char**) controller.jointNamesList.data(); }

size_t GetAxesNumber() { return (size_t) controller.axisNamesList.size(); }

const char** GetAxisNamesList() { return (const char**) controller.axisNamesList.data(); }

size_t GetExtraInputsNumber( void ) { return controller.osimModel->getMuscles().getSize(); }
      
void SetExtraInputsList( double* inputsList ) 
{ 
  controller.emgInputs = SimTK::Vector( controller.osimModel->getMuscles().getSize(), inputsList );
}

size_t GetExtraOutputsNumber( void ) { return 0; }
         
void GetExtraOutputsList( double* outputsList ) { }

void SetControlState( enum RobotState newControlState )
{ 
  std::cout << "setting new control stateFD: " << newControlState;

  const OpenSim::ForceSet &forceSet = controller.osimModel->getForceSet();
  for( int forceIndex = 0; forceIndex < forceSet.getSize(); forceIndex++ )
#ifdef OSIM_LEGACY
    forceSet[ forceIndex ].setDisabled( controller.stateFD, true );
#else
    forceSet[ forceIndex ].setAppliesForce( controller.stateFD, false );
#endif

  if( newControlState == ROBOT_OFFSET )
  {
    std::cout << "starting offset phase" << std::endl;
  }
  else if( newControlState == ROBOT_CALIBRATION )
  {
    std::cout << "starting calibration phase" << std::endl;
  }
  else if( newControlState == ROBOT_PREPROCESSING )
  {
    std::cout << "reseting sampling count" << std::endl;
    controller.emgProcessor->ResetSamplesStorage();
  }
  else 
  {
    if( newControlState == ROBOT_OPERATION )
    {
      if( controller.currentControlState == ROBOT_PREPROCESSING )
      {
        std::cout << "starting optimization" << std::endl;
        SimTK::Vector parametersList = controller.emgProcessor->GetInitialParameters();
        SimTK::Optimizer optimizer( *(controller.emgProcessor), SimTK::LBFGSB );
        optimizer.setConvergenceTolerance( 0.05 );
        optimizer.useNumericalGradient( true );
        optimizer.setMaxIterations( 1000 );
        optimizer.setLimitedMemoryHistory( 500 );
        SimTK::Real remainingError = optimizer.optimize( parametersList );
        std::cout << "optimization ended with residual: " << remainingError << std::endl;
        controller.emgProcessor->SetParameters( parametersList );

        for( int forceIndex = 0; forceIndex < forceSet.getSize(); forceIndex++ )
#ifdef OSIM_LEGACY
          forceSet[ forceIndex ].setDisabled( controller.stateFD, false );
#else
          forceSet[ forceIndex ].setAppliesForce( controller.stateFD, true );
#endif
      }
    }
  }

  controller.currentControlState = newControlState;
}


void PreProcessSample( SimTK::Vector& inputSample, SimTK::Vector& outputSample )
{
  OpenSim::InverseDynamicsSolver idSolver( *(controller.osimModel) );

  SimTK::Vector accelerationsList( controller.osimModel->getCoordinateSet().getSize(), 0.0 );
  for( size_t jointIndex = 0; jointIndex < controller.actuatorsList.size(); jointIndex++ )
  {
    OpenSim::Coordinate* jointCoordinate = controller.actuatorsList[ jointIndex ]->getCoordinate();
    int actuatorInputsIndex = controller.emgInputs.size() + jointIndex * EMG_INPUT_VARS_NUMBER;
    jointCoordinate->setValue( controller.stateFD, inputSample[ actuatorInputsIndex + EMG_POSITION ] );
    jointCoordinate->setSpeedValue( controller.stateFD, inputSample[ actuatorInputsIndex + EMG_VELOCITY ] );
    int jointAccelerationIndex = controller.accelerationIndexesList[ jointIndex ];
    accelerationsList[ jointAccelerationIndex ] = inputSample[ actuatorInputsIndex + EMG_ACCELERATION ];
#ifdef OSIM_LEGACY
    controller.actuatorsList[ jointIndex ]->setOverrideForce( controller.stateFD, inputSample[ actuatorInputsIndex + EMG_TORQUE_EXT ] );
#else
    controller.actuatorsList[ jointIndex ]->setOverrideActuation( controller.stateFD, inputSample[ actuatorInputsIndex + EMG_TORQUE_EXT ] );
#endif
  }
  
  try
  {
    SimTK::Vector idForcesList = idSolver.solve( controller.stateFD, accelerationsList );
    
    for( int jointIndex = 0; jointIndex < controller.actuatorsList.size(); jointIndex++ )
    {
      int actuatorInputsIndex = controller.emgInputs.size() + jointIndex * EMG_INPUT_VARS_NUMBER;
      double positionError = inputSample[ actuatorInputsIndex + EMG_SETPOINT ] - inputSample[ actuatorInputsIndex + EMG_POSITION ];
      int jointTorqueIndex = controller.accelerationIndexesList[ jointIndex ];
      std::cout << "joint " << jointIndex << " coordinate index: " << jointTorqueIndex << std::endl;
      int actuatorOutputsIndex = jointIndex * EMG_OUTPUT_VARS_NUMBER;
      outputSample[ actuatorOutputsIndex + EMG_TORQUE_INT ] = idForcesList[ jointIndex ];
      outputSample[ actuatorOutputsIndex + EMG_STIFFNESS ] = ( std::abs( positionError ) > 1.0e-6 ) ? idForcesList[ jointIndex ] / ( positionError ) : 100.0;
    }
  }
  catch( OpenSim::Exception ex )
  {
    std::cout << ex.getMessage() << std::endl;
  }
  catch( std::exception ex )
  {
    std::cout << ex.what() << std::endl;
  }
}

void RunControlStep( RobotVariables** jointMeasuresList, RobotVariables** axisMeasuresList, RobotVariables** jointSetpointsList, RobotVariables** axisSetpointsList, double timeDelta )
{
  controller.stateFD.updTime() = 0.0;

  SimTK::Vector actuatorInputs( EMG_INPUT_VARS_NUMBER * controller.actuatorsList.size() );
  SimTK::Vector actuatorOutputs( EMG_OUTPUT_VARS_NUMBER * controller.actuatorsList.size() );
  for( size_t jointIndex = 0; jointIndex < controller.actuatorsList.size(); jointIndex++ )
  {
    size_t actuatorInputsIndex = jointIndex * EMG_INPUT_VARS_NUMBER;
    actuatorInputs[ actuatorInputsIndex + EMG_POSITION ] = jointMeasuresList[ jointIndex ]->position;
    actuatorInputs[ actuatorInputsIndex + EMG_VELOCITY ] = jointMeasuresList[ jointIndex ]->velocity;
    actuatorInputs[ actuatorInputsIndex + EMG_ACCELERATION ] = jointMeasuresList[ jointIndex ]->acceleration;
    actuatorInputs[ actuatorInputsIndex + EMG_SETPOINT ] = jointMeasuresList[ jointIndex ]->acceleration;
    actuatorInputs[ actuatorInputsIndex + EMG_TORQUE_EXT ] = jointMeasuresList[ jointIndex ]->force;
  }
  
  PreProcessSample( actuatorInputs, actuatorOutputs );
  
  if( controller.currentControlState == ROBOT_PREPROCESSING )
    controller.emgProcessor->StoreSamples( controller.emgInputs, actuatorInputs, actuatorOutputs );
  else if( controller.currentControlState == ROBOT_OPERATION )
    actuatorOutputs = controller.emgProcessor->CalculateOutputs( controller.emgInputs, actuatorInputs );
  
  for( size_t jointIndex = 0; jointIndex < controller.actuatorsList.size(); jointIndex++ )
  {
    OpenSim::Coordinate* jointCoordinate = controller.actuatorsList[ jointIndex ]->getCoordinate();
    jointCoordinate->setValue( controller.stateFD, jointMeasuresList[ jointIndex ]->position );
    jointCoordinate->setSpeedValue( controller.stateFD, jointMeasuresList[ jointIndex ]->velocity );
    size_t actuatorOutputsIndex = jointIndex * EMG_OUTPUT_VARS_NUMBER;
    double resultingTorque = jointMeasuresList[ jointIndex ]->force + actuatorOutputs[ actuatorOutputsIndex + EMG_TORQUE_INT ];
    controller.actuatorsList[ jointIndex ]->setOverrideActuation( controller.stateFD, resultingTorque );
  }
  
//   OpenSim::Manager manager( *(controller.osimModel) );
// #ifdef OSIM_LEGACY
//   manager.integrate( controller.stateFD, timeDelta );
// #else
//   controller.stateFD = manager.integrate( timeDelta );
// #endif
  
  OpenSim::TimeSeriesTableVec3 markersTimeTable( std::vector<double>( { 0.0 } ) );
  SimTK::RowVector_<SimTK::Vec3> markersTimeRow( controller.markers.getSize() );
  for( int markerIndex = 0; markerIndex < controller.markers.getSize(); markerIndex++ )
  {
    size_t markerAxisIndex = controller.markerAxes[ markerIndex ];
    axisMeasuresList[ markerIndex ]->position = controller.markers[ markerIndex ].getLocationInGround( controller.stateFD )[ markerAxisIndex ];
    axisMeasuresList[ markerIndex ]->velocity = controller.markers[ markerIndex ].getVelocityInGround( controller.stateFD )[ markerAxisIndex ];
    axisMeasuresList[ markerIndex ]->acceleration = controller.markers[ markerIndex ].getAccelerationInGround( controller.stateFD )[ markerAxisIndex ];
    //axisMeasuresTable[ markerIndex ]->force = jointMeasuresTable[ markerIndex ]->force;
    
    markersTimeRow[ markerIndex ][ markerAxisIndex ] = axisSetpointsList[ markerIndex ]->position;
  }
  markersTimeTable.setRow( 0.0, markersTimeRow );
  OpenSim::MarkersReference markersReference( markersTimeTable, &(controller.markerWeights) );
  OpenSim::InverseKinematicsSolver ikSolver( *(controller.osimModel), markersReference, controller.coordinateReferences );
  ikSolver.setAccuracy( 1.0e-4 ); std::cout << "OSim: IK solver set up" << std::endl;
  ikSolver.assemble( controller.stateFD ); std::cout << "OSim: IK solver assembled" << std::endl;
  ikSolver.track( controller.stateFD );
  
  OpenSim::TimeSeriesTableVec3 markersTimeTable( std::vector<double>( { ikState.getTime() } ), markersTable, controller.markerLabels );
  OpenSim::MarkersReference markersReference( markersTimeTable, &(controller.markerWeights) );
  OpenSim::InverseKinematicsSolver ikSolver( osimModel, markersReference, coordinateReferences, 0.0 );
  ikSolver.setAccuracy( 1.0e-4 ); //std::cout << "OSim: IK solver set up" << std::endl;
  ikSolver.assemble( ikState ); //std::cout << "OSim: IK solver assembled" << std::endl;
  ikSolver.track( ikState );
  for( size_t jointIndex = 0; jointIndex < controller.actuatorsList.size(); jointIndex++ )
  {
    OpenSim::Coordinate* jointCoordinate = actuatorsList[ jointIndex ]->getCoordinate();
    jointCoordinate->setValue( state, jointCoordinate->getValue( ikState ) );
    //jointCoordinate->setSpeedValue( state, jointCoordinate->getSpeedValue( ikState ) );
    //jointCoordinate->setAccelerationValue( state, jointCoordinate->getAccelerationValue( ikState ) );
  }
  
  for( size_t jointIndex = 0; jointIndex < controller.actuatorsList.size(); jointIndex++ )
  {
    OpenSim::Coordinate* jointCoordinate = controller.actuatorsList[ jointIndex ]->getCoordinate();
    jointSetpointsList[ jointIndex ]->position = jointCoordinate->getValue( controller.stateFD );
    size_t actuatorOutputsIndex = jointIndex * EMG_OUTPUT_VARS_NUMBER;
    jointSetpointsList[ jointIndex ]->force = actuatorOutputs[ actuatorOutputsIndex + EMG_TORQUE_INT ];
  }
}
