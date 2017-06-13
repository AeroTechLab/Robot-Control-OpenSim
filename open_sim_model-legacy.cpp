#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Actuators/CoordinateActuator.h>

#include <iostream>
#include <string>

#include "robot_control_interface.h"

#include "sensors.h"
#include "data_io.h"
#include "data_logging.h"

#include "emg_optimizer_system.h"

typedef struct _ControlData
{
  OpenSim::Model* osimModel;
  //SimTK::Integrator* integrator;
  //OpenSim::Manager* manager;
  SimTK::Array_<OpenSim::CoordinateActuator*> actuatorsList;
  SimTK::Array_<char*> jointNamesList;
  SimTK::Array_<char*> axisNamesList;
  SimTK::Array_<bool> dofsChangedList;
  enum RobotState currentControlState;
  SimTK::Array_<Sensor> emgSensorsList;
  EMGOptimizerSystem* emgProcessor;
}
ControlData;


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE )


RobotController InitController( const char* data )
{
  DataIO.SetDataType( "JSON" );
  
  ControlData* newController = new ControlData;
  
  try 
  {
    // Create an OpenSim model from XML (.osim) file
    newController->osimModel = new OpenSim::Model( std::string( "config/robots/" ) + std::string( data ) + ".osim" );
    newController->osimModel->printBasicInfo( std::cout );
    
    newController->osimModel->setGravity( SimTK::Vec3( 0.0, -9.80665, 0.0 ) );
    
    //newController->osimModel->setUseVisualizer( true ); // not for RT

    // Initialize the system
    SimTK::State& state = newController->osimModel->initSystem();

    OpenSim::Set<OpenSim::Muscle> muscleSet = newController->osimModel->getMuscles();
    OpenSim::Set<OpenSim::Actuator>& actuatorSet = newController->osimModel->updActuators();
    for( int actuatorIndex = 0; actuatorIndex < actuatorSet.getSize(); actuatorIndex++ )
    {
      std::string actuatorName = actuatorSet[ actuatorIndex ].getName();
      if( muscleSet.contains( actuatorName ) )
      {
        std::string sensorConfigurationPath = std::string( "sensors/emg/" ) + actuatorName;
        DataHandle sensorConfiguration = DataIO.GetDataHandler()->LoadFileData( sensorConfigurationPath.c_str() );
        newController->emgSensorsList.push_back( Sensors.Init( sensorConfiguration ) );
        DataIO.GetDataHandler()->UnloadData( sensorConfiguration );
      }
	  else
      {
        OpenSim::CoordinateActuator* actuator = dynamic_cast<OpenSim::CoordinateActuator*>(&(actuatorSet[ actuatorIndex ]));
		if( actuator != NULL )
		{
		  OpenSim::Coordinate& actuatorCoordinate = newController->osimModel->updCoordinateSet().get( actuator->getName() );
		  actuator->setCoordinate( &actuatorCoordinate );
          newController->actuatorsList.push_back( actuator );
          newController->jointNamesList.push_back( (char*) actuator->getCoordinate()->getName().c_str() );
          newController->axisNamesList.push_back( (char*) actuator->getCoordinate()->getName().c_str() );
          newController->dofsChangedList.push_back( true );
		}
      }
    }
    
    int parametersNumber = EMG_OPT_VARS_NUMBER;
    newController->emgProcessor = new EMGOptimizerSystem( *(newController->osimModel), newController->actuatorsList, 1000 );
    
    SetControlState( newController, /*ROBOT_PASSIVE*/ROBOT_PREPROCESSING );
    
    // Create the integrator and manager for the simulation.
    //newController->integrator = new SimTK::RungeKuttaMersonIntegrator( newController->osimModel->getMultibodySystem() );
    //newController->integrator->setAccuracy( 1.0e-4 ); std::cout << "OSim: integrator created" << std::endl;
    //newController->manager = new OpenSim::Manager( *(newController->osimModel), *(newController->integrator) ); 
    
    std::cout << "OSim: integration manager created" << std::endl;
  }
  catch( OpenSim::Exception ex )
  {
    std::cout << ex.getMessage() << std::endl;
    EndController( (RobotController) newController );
    return NULL;
  }
  catch( std::exception ex )
  {
    std::cout << ex.what() << std::endl;
    EndController( (RobotController) newController );
    return NULL;
  }
  catch( ... )
  {
    std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
    EndController( (RobotController) newController );
    return NULL;
  }
  
  std::cout << "OpenSim model loaded successfully ! (" << newController->osimModel->getNumCoordinates() << " coordinates)" << std::endl;
  
  return (RobotController) newController;
}

void EndController( RobotController ref_controller )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  //delete controller->integrator;
  //delete controller->manager;
  delete controller->osimModel;
  
  controller->jointNamesList.clear();
  controller->axisNamesList.clear();
  controller->dofsChangedList.clear();
  
  for( size_t muscleIndex = 0; muscleIndex < controller->emgSensorsList.size(); muscleIndex++ )
    Sensors.End( controller->emgSensorsList[ muscleIndex ] );
  controller->emgSensorsList.clear();
  
  delete controller;
}

size_t GetJointsNumber( RobotController ref_controller )
{
  if( ref_controller == NULL ) return 0;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return (size_t) controller->jointNamesList.size();
}

const char** GetJointNamesList( RobotController ref_controller )
{
  if( ref_controller == NULL ) return NULL;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return (const char**) controller->jointNamesList.data();
}

const bool* GetJointsChangedList( RobotController ref_controller )
{
  if( ref_controller == NULL ) return NULL;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return (const bool*) controller->dofsChangedList.data();
}

size_t GetAxesNumber( RobotController ref_controller )
{
  if( ref_controller == NULL ) return 0;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return (size_t) controller->axisNamesList.size();
}

const char** GetAxisNamesList( RobotController ref_controller )
{
  if( ref_controller == NULL ) return NULL;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return (const char**) controller->axisNamesList.data();
}

const bool* GetAxesChangedList( RobotController ref_controller )
{
  if( ref_controller == NULL ) return NULL;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return (const bool*) controller->dofsChangedList.data();
}

void SetControlState( RobotController ref_controller, enum RobotState newControlState )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  std::cout << "setting new control state: " << newControlState;
  
  SimTK::State& state = controller->osimModel->initSystem();

  enum SignalProcessingPhase signalProcessingPhase = SIGNAL_PROCESSING_PHASE_MEASUREMENT;

  OpenSim::ForceSet &forceSet = controller->osimModel->updForceSet();
  for( int forceIndex = 0; forceIndex < forceSet.getSize(); forceIndex++ )
	forceSet[ forceIndex ].setDisabled( state, true ); // setAppliesForce -> false

  if( newControlState == ROBOT_OFFSET )
  {
	std::cout << "starting offset phase" << std::endl;
	signalProcessingPhase = SIGNAL_PROCESSING_PHASE_OFFSET;
  }
  else if( newControlState == ROBOT_CALIBRATION )
  {
	std::cout << "starting calibration phase" << std::endl;
	signalProcessingPhase = SIGNAL_PROCESSING_PHASE_CALIBRATION;
  }
  else if( newControlState == ROBOT_PREPROCESSING )
  {
	std::cout << "reseting sampling count" << std::endl;
	controller->emgProcessor->ResetSamplesStorage();
  }
  else 
  {
	if( newControlState == ROBOT_OPERATION )
	{
	  if( controller->currentControlState == ROBOT_PREPROCESSING )
	  {
	    std::cout << "starting optimization" << std::endl;
		controller->emgProcessor->PreProcessSamples();
		SimTK::Vector parametersList = controller->emgProcessor->GetInitialParameters();
		SimTK::Optimizer optimizer( *(controller->emgProcessor), SimTK::LBFGSB );
		optimizer.setConvergenceTolerance( 0.05 );
		optimizer.useNumericalGradient( true );
		optimizer.setMaxIterations( 1000 );
		optimizer.setLimitedMemoryHistory( 500 );
		SimTK::Real remainingError = optimizer.optimize( parametersList );
		std::cout << "optimization ended with residual: " << remainingError << std::endl;

		OpenSim::ForceSet &forceSet = controller->osimModel->updForceSet();
		for( int forceIndex = 0; forceIndex < forceSet.getSize(); forceIndex++ )
		  forceSet[ forceIndex ].setDisabled( state, false ); // setAppliesForce -> true
	  }
	}
  }
  
  for( size_t muscleIndex = 0; muscleIndex < controller->emgSensorsList.size(); muscleIndex++ )
	Sensors.SetState( controller->emgSensorsList[ muscleIndex ], signalProcessingPhase );

  controller->currentControlState = newControlState;
}

void RunControlStep( RobotController RobotController, RobotVariables** jointMeasuresList, RobotVariables** axisMeasuresList, RobotVariables** jointSetpointsList, RobotVariables** axisSetpointsList, double timeDelta )
{
  static size_t passesCount;

  if( RobotController == NULL ) return;
  
  ControlData* controller = (ControlData*) RobotController;

  SimTK::State& state = controller->osimModel->initSystem();

  state.updTime() = 0.0;

  SimTK::Vector emgInputs( controller->emgSensorsList.size() );
  for( size_t muscleIndex = 0; muscleIndex < controller->emgSensorsList.size(); muscleIndex++ )
    emgInputs[ muscleIndex ] = Sensors.Update( controller->emgSensorsList[ muscleIndex ], NULL );
  SimTK::Vector positionInputs( EMG_POS_VARS_NUMBER * controller->actuatorsList.size() );
  SimTK::Vector torqueInputs( EMG_FORCE_VARS_NUMBER * controller->actuatorsList.size() );
  for( size_t jointIndex = 0; jointIndex < controller->actuatorsList.size(); jointIndex++ )
  {
	OpenSim::Coordinate* jointCoordinate = controller->actuatorsList[ jointIndex ]->getCoordinate();
    jointCoordinate->setValue( state, jointMeasuresList[ jointIndex ]->position );
    jointCoordinate->setSpeedValue( state, jointMeasuresList[ jointIndex ]->velocity );
    controller->actuatorsList[ jointIndex ]->setForce( state, jointMeasuresList[ jointIndex ]->force );
    size_t positionInputsIndex = jointIndex * EMG_POS_VARS_NUMBER;
    positionInputs[ positionInputsIndex + EMG_ANGLE ] = jointMeasuresList[ jointIndex ]->position;
    positionInputs[ positionInputsIndex + EMG_VELOCITY ] = jointMeasuresList[ jointIndex ]->velocity;
    positionInputs[ positionInputsIndex + EMG_ACCELERATION ] = jointMeasuresList[ jointIndex ]->acceleration;
    positionInputs[ positionInputsIndex + EMG_SETPOINT ] = jointMeasuresList[ jointIndex ]->acceleration;
    size_t torqueOutputsIndex = jointIndex * EMG_FORCE_VARS_NUMBER;
    torqueInputs[ torqueOutputsIndex + EMG_TORQUE_EXT ] = jointMeasuresList[ jointIndex ]->force;
  }
  
  if( controller->currentControlState == ROBOT_PREPROCESSING )
    controller->emgProcessor->StoreSamples( emgInputs, positionInputs, torqueInputs );

  //else if( controller->currentControlState == ROBOT_OPERATION )
  SimTK::Vector torqueOutputs = controller->emgProcessor->CalculateTorques( state, emgInputs );
  
  //SimTK::RungeKuttaMersonIntegrator integrator ( controller->osimModel->getMultibodySystem() );
  //integrator.setAccuracy( 1.0e-6 );
  //OpenSim::Manager manager( *(controller->osimModel), integrator );

  //manager.setInitialTime( 0.0 );
  //manager.setFinalTime( timeDelta );
  controller->osimModel->getMultibodySystem().realize( state, SimTK::Stage::Dynamics );
  //manager.integrate( state );
  
  for( size_t axisIndex = 0; axisIndex < controller->actuatorsList.size(); axisIndex++ )
  {
    OpenSim::Coordinate* axisCoordinate = controller->actuatorsList[ axisIndex ]->getCoordinate();
    axisMeasuresList[ axisIndex ]->position = axisCoordinate->getValue( state );
    axisMeasuresList[ axisIndex ]->velocity = axisCoordinate->getSpeedValue( state );
    axisMeasuresList[ axisIndex ]->acceleration = axisCoordinate->getAccelerationValue( state );
    size_t torqueOutputsIndex = axisIndex * EMG_FORCE_VARS_NUMBER;
    axisMeasuresList[ axisIndex ]->force = torqueOutputs[ torqueOutputsIndex + EMG_TORQUE_EXT ];
    axisMeasuresList[ axisIndex ]->stiffness = torqueOutputs[ torqueOutputsIndex + EMG_STIFFNESS ];
  }
  
  for( size_t jointIndex = 0; jointIndex < controller->actuatorsList.size(); jointIndex++ )
  {
    jointSetpointsList[ jointIndex ]->position = axisSetpointsList[ jointIndex ]->position;
    jointSetpointsList[ jointIndex ]->velocity = axisSetpointsList[ jointIndex ]->velocity;
    jointSetpointsList[ jointIndex ]->acceleration = axisSetpointsList[ jointIndex ]->acceleration;
    jointSetpointsList[ jointIndex ]->force = axisSetpointsList[ jointIndex ]->force;
  }

  if( passesCount++ > 100 && controller->currentControlState == ROBOT_PREPROCESSING ) SetControlState( controller, ROBOT_OPERATION );

  //std::cout << "joint 0 position: " << controller->actuatorsList[ 0 ]->getCoordinate()->getValue( state ) << std::endl;
}
