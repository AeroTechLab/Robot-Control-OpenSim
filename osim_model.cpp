#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Actuators/CoordinateActuator.h>

#include <iostream>
#include <string>

#include "interface/robot_control.h"

#include "emg_optimizer_system.h"

struct
{
  OpenSim::Model* osimModel;
  SimTK::Array_<OpenSim::CoordinateActuator*> actuatorsList;
  SimTK::Array_<char*> jointNamesList;
  SimTK::Array_<char*> axisNamesList;
  enum RobotState currentControlState;
  SimTK::Vector emgInputs;
  EMGOptimizerSystem* emgProcessor;
}
controller;


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


bool InitController( const char* data )
{ 
  try 
  {
    // Create an OpenSim model from XML (.osim) file
    controller.osimModel = new OpenSim::Model( std::string( "config/robots/" ) + std::string( data ) + ".osim" );
    controller.osimModel->printBasicInfo( std::cout );
    
    controller.osimModel->setGravity( SimTK::Vec3( 0.0, -9.80665, 0.0 ) );
    
    //controller.osimModel->setUseVisualizer( true ); // not for RT

    // Initialize the system
    SimTK::State& state = controller.osimModel->initSystem();

    OpenSim::Set<OpenSim::Muscle> muscleSet = controller.osimModel->getMuscles();
    OpenSim::Set<OpenSim::Actuator>& actuatorSet = controller.osimModel->updActuators();
    for( int actuatorIndex = 0; actuatorIndex < actuatorSet.getSize(); actuatorIndex++ )
    {
      std::string actuatorName = actuatorSet[ actuatorIndex ].getName();
      if( not muscleSet.contains( actuatorName ) )
      {
        OpenSim::CoordinateActuator* actuator = dynamic_cast<OpenSim::CoordinateActuator*>(&(actuatorSet[ actuatorIndex ]));
        if( actuator != NULL )
        {
          OpenSim::Coordinate& actuatorCoordinate = controller.osimModel->updCoordinateSet().get( actuator->getName() );
          actuator->setCoordinate( &actuatorCoordinate );
          controller.actuatorsList.push_back( actuator );
          controller.jointNamesList.push_back( (char*) actuator->getCoordinate()->getName().c_str() );
          controller.axisNamesList.push_back( (char*) actuator->getCoordinate()->getName().c_str() );
        }
      }
    }
    
    int parametersNumber = EMG_OPT_VARS_NUMBER;
    controller.emgProcessor = new EMGOptimizerSystem( *(controller.osimModel), controller.actuatorsList, 1000 );
    
    SetControlState( /*ROBOT_PASSIVE*/ROBOT_PREPROCESSING );
    
    std::cout << "OSim: integration manager created" << std::endl;
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
  
  std::cout << "OpenSim model loaded successfully ! (" << controller.osimModel->getNumCoordinates() << " coordinates)" << std::endl;
  
  return true;
}

void EndController()
{
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
  std::cout << "setting new control state: " << newControlState;
  
  SimTK::State& state = controller.osimModel->initSystem();

  OpenSim::ForceSet &forceSet = controller.osimModel->updForceSet();
  for( int forceIndex = 0; forceIndex < forceSet.getSize(); forceIndex++ )
#ifdef OSIM_LEGACY
    forceSet[ forceIndex ].setDisabled( state, true );
#else
    forceSet[ forceIndex ].setAppliesForce( state, false );
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
        controller.emgProcessor->PreProcessSamples();
        SimTK::Vector parametersList = controller.emgProcessor->GetInitialParameters();
        SimTK::Optimizer optimizer( *(controller.emgProcessor), SimTK::LBFGSB );
        optimizer.setConvergenceTolerance( 0.05 );
        optimizer.useNumericalGradient( true );
        optimizer.setMaxIterations( 1000 );
        optimizer.setLimitedMemoryHistory( 500 );
        SimTK::Real remainingError = optimizer.optimize( parametersList );
        std::cout << "optimization ended with residual: " << remainingError << std::endl;

        OpenSim::ForceSet &forceSet = controller.osimModel->updForceSet();
        for( int forceIndex = 0; forceIndex < forceSet.getSize(); forceIndex++ )
#ifdef OSIM_LEGACY
          forceSet[ forceIndex ].setDisabled( state, false );
#else
          forceSet[ forceIndex ].setAppliesForce( state, true );
#endif
      }
    }
  }

  controller.currentControlState = newControlState;
}

void RunControlStep( RobotVariables** jointMeasuresList, RobotVariables** axisMeasuresList, RobotVariables** jointSetpointsList, RobotVariables** axisSetpointsList, double timeDelta )
{
  SimTK::State& state = controller.osimModel->initSystem();

  state.updTime() = 0.0;

  SimTK::Vector positionInputs( EMG_POS_VARS_NUMBER * controller.actuatorsList.size() );
  SimTK::Vector torqueInputs( EMG_FORCE_VARS_NUMBER * controller.actuatorsList.size() );
  for( size_t jointIndex = 0; jointIndex < controller.actuatorsList.size(); jointIndex++ )
  {
	OpenSim::Coordinate* jointCoordinate = controller.actuatorsList[ jointIndex ]->getCoordinate();
    jointCoordinate->setValue( state, jointMeasuresList[ jointIndex ]->position );
    jointCoordinate->setSpeedValue( state, jointMeasuresList[ jointIndex ]->velocity );
#ifdef OSIM_LEGACY
    controller.actuatorsList[ jointIndex ]->setForce( state, jointMeasuresList[ jointIndex ]->force );
#else
    controller.actuatorsList[ jointIndex ]->setActuation( state, jointMeasuresList[ jointIndex ]->force );
#endif
    size_t positionInputsIndex = jointIndex * EMG_POS_VARS_NUMBER;
    positionInputs[ positionInputsIndex + EMG_ANGLE ] = jointMeasuresList[ jointIndex ]->position;
    positionInputs[ positionInputsIndex + EMG_VELOCITY ] = jointMeasuresList[ jointIndex ]->velocity;
    positionInputs[ positionInputsIndex + EMG_ACCELERATION ] = jointMeasuresList[ jointIndex ]->acceleration;
    positionInputs[ positionInputsIndex + EMG_SETPOINT ] = jointMeasuresList[ jointIndex ]->acceleration;
    size_t torqueOutputsIndex = jointIndex * EMG_FORCE_VARS_NUMBER;
    torqueInputs[ torqueOutputsIndex + EMG_TORQUE_EXT ] = jointMeasuresList[ jointIndex ]->force;
  }
  
  if( controller.currentControlState == ROBOT_PREPROCESSING )
    controller.emgProcessor->StoreSamples( controller.emgInputs, positionInputs, torqueInputs );

  SimTK::Vector torqueOutputs = controller.emgProcessor->CalculateTorques( state, controller.emgInputs );
  
  OpenSim::Manager manager( *(controller.osimModel) );
#ifdef OSIM_LEGACY
  manager.integrate( state, timeDelta );
#else
  state = manager.integrate( timeDelta );
#endif
  
  for( size_t axisIndex = 0; axisIndex < controller.actuatorsList.size(); axisIndex++ )
  {
    OpenSim::Coordinate* axisCoordinate = controller.actuatorsList[ axisIndex ]->getCoordinate();
    axisMeasuresList[ axisIndex ]->position = axisCoordinate->getValue( state );
    axisMeasuresList[ axisIndex ]->velocity = axisCoordinate->getSpeedValue( state );
    axisMeasuresList[ axisIndex ]->acceleration = axisCoordinate->getAccelerationValue( state );
    size_t torqueOutputsIndex = axisIndex * EMG_FORCE_VARS_NUMBER;
    axisMeasuresList[ axisIndex ]->force = torqueOutputs[ torqueOutputsIndex + EMG_TORQUE_EXT ];
    axisMeasuresList[ axisIndex ]->stiffness = torqueOutputs[ torqueOutputsIndex + EMG_STIFFNESS ];
  }
  
  for( size_t jointIndex = 0; jointIndex < controller.actuatorsList.size(); jointIndex++ )
  {
    jointSetpointsList[ jointIndex ]->position = axisSetpointsList[ jointIndex ]->position;
    jointSetpointsList[ jointIndex ]->velocity = axisSetpointsList[ jointIndex ]->velocity;
    jointSetpointsList[ jointIndex ]->acceleration = axisSetpointsList[ jointIndex ]->acceleration;
    jointSetpointsList[ jointIndex ]->force = axisSetpointsList[ jointIndex ]->force;
  }

  //std::cout << "joint 0 position: " << controller.actuatorsList[ 0 ]->getCoordinate()->getValue( state ) << std::endl;
}
