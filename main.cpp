#include <iostream>
#include <string>
#include <cmath>

//#include "emg_processing.h"
#include "sensors.h"
#include "neural_network.h"

#include "robot_control_interface.h"
#include "data_io.h"
#include "data_logging.h"

#include "timing.h"

#define OUTPUTS_NUMBER 2

#define HIDDEN_NEURONS_NUMBER 10

#define SAMPLES_NUMBER 1000

#define DOFS_NUMBER 1
const char* DOF_NAMES[ DOFS_NUMBER ] = { "Angle" };

const bool DOFS_CHANGED_LIST[ DOFS_NUMBER ] = { true };

typedef struct _ControlData
{
  double* inputsList;
  double outputsList[ OUTPUTS_NUMBER ];
  double** inputSamplesTable;
  double** outputSamplesTable;
  size_t samplesCount;
  enum RobotState currentControlState;
  Sensor* emgSensorsList;
  size_t musclesCount;
  NeuralNetwork modelNetwork;
  Log log;
  bool dofChanged;
}
ControlData;


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE )


RobotController InitController( const char* configurationString )
{
  std::cout << "Trying to load robot control config " << configurationString << std::endl;
  
  DataIO.SetDataType( "JSON" );

  DataLogging.SetBaseDirectory( "test" );
  
  ControlData* newController = NULL;

  DataHandle configuration = DataIO.GetDataHandler()->LoadStringData( configurationString );
  if( configuration != NULL )
  {   
	newController = new ControlData;
    memset( newController, 0, sizeof(ControlData) );
  
	if( (newController->musclesCount = (size_t) DataIO.GetDataHandler()->GetListSize( configuration, "muscles" )) > 0 )
	{
	  newController->emgSensorsList = new Sensor[ newController->musclesCount ];
      for( size_t muscleIndex = 0; muscleIndex < newController->musclesCount; muscleIndex++ )
      {
        DataHandle sensorConfiguration = DataIO.GetDataHandler()->GetSubData( configuration, "muscles.%lu", muscleIndex );
		newController->emgSensorsList[ muscleIndex ] = Sensors.Init( sensorConfiguration );
      }
	}

	newController->log = DataLogging.InitLog( "neural_network", 6 );
    
    DataIO.GetDataHandler()->UnloadData( configuration );

    std::cout << "robot control config " << configurationString << " loaded";

	newController->inputsList = new double[ newController->musclesCount + 1 ];

    newController->inputSamplesTable = new double*[ SAMPLES_NUMBER ];
	newController->outputSamplesTable = new double*[ SAMPLES_NUMBER ];
	for( size_t sampleIndex = 0; sampleIndex < SAMPLES_NUMBER; sampleIndex++ )
	{
	  newController->inputSamplesTable[ sampleIndex ] = new double[ newController->musclesCount + 1 ];
	  newController->outputSamplesTable[ sampleIndex ] = new double[ OUTPUTS_NUMBER ];
	}

	newController->currentControlState = ROBOT_PASSIVE;

	newController->modelNetwork = NeuralNetworks.InitNetwork( newController->musclesCount + 1, OUTPUTS_NUMBER, HIDDEN_NEURONS_NUMBER );

	newController->dofChanged = true;
  }
  
  return (RobotController) newController;
}

void EndController( RobotController ref_controller )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  std::cout << "ending robot controller " << controller << std::endl;
      
  for( size_t muscleIndex = 0; muscleIndex < controller->musclesCount; muscleIndex++ )
	  Sensors.End( controller->emgSensorsList[ muscleIndex ] );
  delete controller->emgSensorsList;
    
  DataLogging.EndLog( controller->log );
  
  for( size_t sampleIndex = 0; sampleIndex < SAMPLES_NUMBER; sampleIndex++ )
  {
	delete controller->inputSamplesTable[ sampleIndex ];
	delete controller->outputSamplesTable[ sampleIndex ];
  }
  delete controller->inputSamplesTable;
  delete controller->outputSamplesTable;

  delete controller->inputsList;

  NeuralNetworks.EndNetwork( controller->modelNetwork );
}

size_t GetJointsNumber( RobotController RobotController )
{
  if( RobotController == NULL ) return 0;
  
  ControlData* controller = (ControlData*) RobotController;
  
  return 1;
}

const char** GetJointNamesList( RobotController RobotController )
{
  if( RobotController == NULL ) return NULL;
  
  ControlData* controller = (ControlData*) RobotController;
  
  return (const char**) DOF_NAMES;
}

const bool* GetJointsChangedList( RobotController ref_controller )
{
  if( ref_controller == NULL ) return NULL;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return (const bool*) &(controller->dofChanged);
}

size_t GetAxesNumber( RobotController RobotController )
{
  if( RobotController == NULL ) return 0;
  
  ControlData* controller = (ControlData*) RobotController;
  
  return 1;
}

const char** GetAxisNamesList( RobotController RobotController )
{
  if( RobotController == NULL ) return NULL;
  
  ControlData* controller = (ControlData*) RobotController;
  
  return (const char**) DOF_NAMES;
}

const bool* GetAxesChangedList( RobotController ref_controller )
{
  if( ref_controller == NULL ) return NULL;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return (const bool*) &(controller->dofChanged);
}

void SetControlState( RobotController ref_controller, enum RobotState newControlState )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  std::cout << "setting new control state: " << newControlState;
  
  enum SignalProcessingPhase signalProcessingPhase = SIGNAL_PROCESSING_PHASE_MEASUREMENT;

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
	controller->samplesCount = 0;
  }
  else 
  {
	if( newControlState == ROBOT_OPERATION )
	{
	  if( controller->currentControlState == ROBOT_PREPROCESSING && controller->samplesCount > 0 )
	  {
	    std::cout << "starting optimization" << std::endl;
		NeuralNetworks.EnterTrainingData( controller->modelNetwork, controller->inputSamplesTable, controller->outputSamplesTable, controller->samplesCount );
	  }
	}
  }
  
  for( size_t muscleIndex = 0; muscleIndex < controller->musclesCount; muscleIndex++ )
	Sensors.SetState( controller->emgSensorsList[ muscleIndex ], signalProcessingPhase );

  controller->currentControlState = newControlState;
}

void RunControlStep( RobotController ref_controller, RobotVariables** jointMeasuresTable, RobotVariables** axisMeasuresTable, RobotVariables** jointSetpointsTable, RobotVariables** axisSetpointsTable, double timeDelta )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  double jointMeasureAngle = jointMeasuresTable[ 0 ]->position * 180.0 / M_PI;
  double jointExternalTorque = jointMeasuresTable[ 0 ]->force;

  axisMeasuresTable[ 0 ]->position = jointMeasuresTable[ 0 ]->position;
  axisMeasuresTable[ 0 ]->velocity = jointMeasuresTable[ 0 ]->velocity;
  axisMeasuresTable[ 0 ]->acceleration = jointMeasuresTable[ 0 ]->acceleration;
    
  jointSetpointsTable[ 0 ]->position = axisSetpointsTable[ 0 ]->position;
  jointSetpointsTable[ 0 ]->velocity = axisSetpointsTable[ 0 ]->velocity;
  jointSetpointsTable[ 0 ]->acceleration = axisSetpointsTable[ 0 ]->acceleration;
    
  double jointSetpointAngle = jointSetpointsTable[ 0 ]->position * 180.0 / M_PI;

  for( size_t muscleIndex = 0; muscleIndex < controller->musclesCount; muscleIndex++ )
	controller->inputsList[ muscleIndex ] = Sensors.Update( controller->emgSensorsList[ muscleIndex ], NULL );

  if( controller->currentControlState == ROBOT_PREPROCESSING )
  {
	  if( controller->samplesCount < SAMPLES_NUMBER )
	  {
		  double* inputSamplesList = controller->inputSamplesTable[ controller->samplesCount ];
		  double* outputSamplesList = controller->outputSamplesTable[ controller->samplesCount ];

		  for( size_t muscleIndex = 0; muscleIndex < controller->musclesCount; muscleIndex++ ) 
			  inputSamplesList[ muscleIndex ] = controller->inputsList[ muscleIndex ];
		  inputSamplesList[ controller->musclesCount ] = jointMeasureAngle;

		  outputSamplesList[ 0 ] = jointExternalTorque;
		  outputSamplesList[ 1 ] = 0.0;//jointExternalTorque / ( jointSetpointAngle - jointMeasureAngle );

		  //DEBUG_PRINT( "Saving sample %u: %.3f, %.3f", sampler->samplesCount, jointMeasuredAngle, jointExternalTorque );

		  controller->samplesCount++;
	  }
  }

  //std::cout << "joint pos=" << jointMeasureAngle << ", force=" << jointExternalTorque << ", stiff=" << jointMeasuresTable[ 0 ]->stiffness << std::endl;

  jointSetpointsTable[ 0 ]->force = axisSetpointsTable[ 0 ]->force;
  jointSetpointsTable[ 0 ]->stiffness = axisSetpointsTable[ 0 ]->stiffness ;
  jointSetpointsTable[ 0 ]->damping = axisSetpointsTable[ 0 ]->damping;

  double setpointStiffness = jointSetpointsTable[ 0 ]->stiffness ;
  double positionError = jointSetpointsTable[ 0 ]->position - jointMeasuresTable[ 0 ]->position;

  if( controller->currentControlState == ROBOT_OPERATION )
  {
	  NeuralNetworks.ProcessInput( controller->modelNetwork, controller->inputsList, controller->outputsList );
	  double jointEMGTorque = controller->outputsList[ 0 ];
	  double jointEMGStiffness = 0.0;//controller->outputsList[ 1 ]

	  axisMeasuresTable[ 0 ]->force = jointEMGTorque;
	  axisMeasuresTable[ 0 ]->stiffness  = jointEMGStiffness;  

	  double targetStiffness = 0.0;//1.0 / jointEMGStiffness;
	  setpointStiffness = 0.99 * setpointStiffness + 0.01 * targetStiffness;

	  jointMeasuresTable[ 0 ]->stiffness  = setpointStiffness;  

	  if( controller->currentControlState == ROBOT_OPERATION )
	  {
		  DataLogging.EnterNewLine( controller->log );
		  DataLogging.RegisterList( controller->log, controller->musclesCount, controller->inputsList );
		  DataLogging.RegisterValues( controller->log, 5, jointMeasureAngle, jointExternalTorque, jointEMGTorque, targetStiffness, jointEMGStiffness );
	  }

	  //std::cout << "emg:" << jointEMGStiffness << " - t:" << targetStiffness << " - s:" << setpointStiffness << std::endl;
  }
  else
  {
	  setpointStiffness = 0.0;
  }

  jointSetpointsTable[ 0 ]->force = setpointStiffness * positionError;
}
