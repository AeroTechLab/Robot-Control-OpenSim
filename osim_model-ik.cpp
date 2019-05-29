#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <simbody/internal/Visualizer_InputListener.h>

#include <iostream>
#include <string>

#include "interface/robot_control.h"

#ifndef USE_NN
  #include "emg_optimizer-nn.h"
#else
  #include "emg_optimizer-osim.h"
#endif

struct
{
  OpenSim::Model* osimModel;
  SimTK::State state;
  SimTK::Array_<OpenSim::CoordinateActuator*> actuatorsList;
  SimTK::Array_<int> accelerationIndexesList;
  OpenSim::InverseKinematicsSolver* ikSolver;
  SimTK::Array_<OpenSim::CoordinateReference> coordinateReferences;
  OpenSim::MarkerSet markers;
  OpenSim::MarkersReference markersReference;
  SimTK::Array_<char*> jointNamesList;
  SimTK::Array_<char*> axisNamesList;
  enum RobotState currentControlState;
  SimTK::Vector emgInputs;
  EMGOptimizerImpl* emgProcessor;
}
controller;


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


bool InitController( const char* data )
{ 
  try 
  {
    // Create an OpenSim model from XML (.osim) file
    controller.osimModel = new OpenSim::Model( std::string( "config/robots/" ) + data + ".osim" );
    controller.osimModel->printBasicInfo( std::cout );
    
    controller.osimModel->setGravity( SimTK::Vec3( 0.0, -9.80665, 0.0 ) );
    
    //controller.osimModel->setUseVisualizer( true ); // not for RT

    // Initialize the system
    controller.state = controller.osimModel->initSystem();

    OpenSim::MarkerSet& markerSet = controller.osimModel->updMarkerSet(); std::cout << "OSim: found " << markerSet.getSize() << " markers" << std::endl;
	OpenSim::MarkerFrame* updateFrame = new OpenSim::MarkerFrame( controller.markerSet.getSize(), 0, 0.0, OpenSim::Units::Meters );
    for( int markerIndex = 0; markerIndex < markerSet.getSize(); markerIndex++ )
    {
      std::string markerName = markerSet[ markerIndex ].getName();
      const char* REFERENCE_AXES_NAMES[ 3 ] = { "_ref_X", "_ref_Y", "_ref_Z" };
      for( size_t referenceAxisIndex = 0; referenceAxisIndex < 3; referenceAxisIndex++ )
      {
        if( markerName.rfind( REFERENCE_AXES_NAMES[ referenceAxisIndex ] ) != std::string::npos )
        {
          std::cout << "OSim: found reference marker " << markerName << std::endl;
		  updateFrame->addMarker( SimTK::Vec3( 0 ) );
          controller.markers.adoptAndAppend( &(markerSet[ markerIndex ]) );
		  controller.markersReference._markerNames.push_back( markerName );
		  controller.markersReference._weights.push_back( 1.0 );
		  controller.markersReference._markerWeightSet.adoptAndAppend( new OpenSim::MarkerWeight( markerName, 1.0 ) );
          controller.markerReferenceAxes.push_back( referenceAxisIndex );
          controller.axisNames.push_back( (char*) markerSet[ markerIndex ].getName().c_str() );
          break;
        }
      }
    }
	controller.markersReference._markerData = new OpenSim::MarkerData();
	controller.markersReference._markerData->_frames.append( updateFrame );
	controller.markersReference._markerData->_numFrames = controller.markersReference._markerData->_frames.getSize();

	controller.axesChangedList.resize( markerSet.getSize() );
    
	std::cout << "OSim: generated markers reference size" << controller.markersReference.updMarkerWeightSet().getSize() << std::endl;
	std::cout << "OSim: generated coordinates reference size: " << controller.coordinateReferences.size() << std::endl;
    controller.ikSolver = new OpenSim::InverseKinematicsSolver( *(controller.osimModel), controller.markersReference, controller.coordinateReferences );
	std::cout << "OSim: IK solver created" << std::endl;
    controller.ikSolver->setAccuracy( 1.0e-4 ); std::cout << "OSim: IK solver set up" << std::endl;
    controller.ikSolver->assemble( controller.state ); std::cout << "OSim: IK solver assembled" << std::endl;
    
    OpenSim::Set<OpenSim::Muscle> muscleSet = controller.osimModel->getMuscles();
    for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
#ifdef OSIM_LEGACY
      muscleSet[ muscleIndex ].setDisabled( controller.state, true );
#else
      muscleSet[ muscleIndex ].setAppliesForce( controller.state, false );
#endif
    OpenSim::Set<OpenSim::Actuator>& actuatorSet = controller.osimModel->updActuators();
    for( int actuatorIndex = 0; actuatorIndex < actuatorSet.getSize(); actuatorIndex++ )
    {
      std::string actuatorName = actuatorSet[ actuatorIndex ].getName();
      if( not muscleSet.contains( actuatorName ) )
      {
        OpenSim::CoordinateActuator* actuator = dynamic_cast<OpenSim::CoordinateActuator*>(&(actuatorSet[ actuatorIndex ]));
        if( actuator != NULL )
        {
#ifdef OSIM_LEGACY
          actuator->overrideForce( controller.state, true );
#else
          actuator->overrideActuation( controller.state, true );
#endif
          OpenSim::CoordinateSet coordinateSet = controller.osimModel->getCoordinateSet();
          OpenSim::Coordinate& actuatorCoordinate = coordinateSet.get( actuator->getName() );
          actuator->setCoordinate( &actuatorCoordinate );
          controller.actuatorsList.push_back( actuator );
          controller.jointNamesList.push_back( (char*) actuator->getCoordinate()->getName().c_str() );
          controller.axisNamesList.push_back( (char*) actuator->getCoordinate()->getName().c_str() );
          controller.accelerationIndexesList.push_back( coordinateSet.getIndex( &actuatorCoordinate ) );
        }
      }
    }
    
    controller.emgProcessor = new EMGOptimizerImpl( *(controller.osimModel), controller.actuatorsList, 1000 );
    
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
  controller.osimModel->markers.clearAndDestroy();
  controller.osimModel->actuatorsList.clear();
  controller.osimModel->accelerationIndexesList.clear();
  controller.osimModel->coordinateReferences.clear(); 
  
  delete controller.osimModel;
  
  delete controller.ikSolver;
  
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

  OpenSim::ForceSet &forceSet = controller.osimModel->updForceSet();
  for( int forceIndex = 0; forceIndex < forceSet.getSize(); forceIndex++ )
#ifdef OSIM_LEGACY
    forceSet[ forceIndex ].setDisabled( controller.state, true );
#else
    forceSet[ forceIndex ].setAppliesForce( controller.state, false );
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

        OpenSim::ForceSet& forceSet = controller.osimModel->updForceSet();
        for( int forceIndex = 0; forceIndex < forceSet.getSize(); forceIndex++ )
#ifdef OSIM_LEGACY
          forceSet[ forceIndex ].setDisabled( controller.state, false );
#else
          forceSet[ forceIndex ].setAppliesForce( controller.state, true );
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
    jointCoordinate->setValue( controller.state, inputSample[ actuatorInputsIndex + EMG_POSITION ] );
    jointCoordinate->setSpeedValue( controller.state, inputSample[ actuatorInputsIndex + EMG_VELOCITY ] );
    int jointAccelerationIndex = controller.accelerationIndexesList[ jointIndex ];
    accelerationsList[ jointAccelerationIndex ] = inputSample[ actuatorInputsIndex + EMG_ACCELERATION ];
#ifdef OSIM_LEGACY
    controller.actuatorsList[ jointIndex ]->setOverrideForce( controller.state, inputSample[ actuatorInputsIndex + EMG_TORQUE_EXT ] );
#else
    controller.actuatorsList[ jointIndex ]->setOverrideActuation( controller.state, inputSample[ actuatorInputsIndex + EMG_TORQUE_EXT ] );
#endif
  }
  
  try
  {
    SimTK::Vector idForcesList = idSolver.solve( controller.state, accelerationsList );
    
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
  controller.state.updTime() = 0.0;

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
  
  OpenSim::Manager manager( *(controller.osimModel) );
#ifdef OSIM_LEGACY
  manager.integrate( controller.state, timeDelta );
#else
  controller.state = manager.integrate( timeDelta );
#endif
  
  SimTK::Vec3 markerPosition, markerVelocity, markerAcceleration;
  for( int markerIndex = 0; markerIndex < controller.markers.getSize(); markerIndex++ )
  {
    controller.osimModel->getSimbodyEngine().getPosition( controller.state, controller.markers[ markerIndex ].getBody(), controller.markers[ markerIndex ].getOffset(), markerPosition );
    controller.osimModel->getSimbodyEngine().getVelocity( controller.state, controller.markers[ markerIndex ].getBody(), controller.markers[ markerIndex ].getOffset(), markerVelocity );
    controller.osimModel->getSimbodyEngine().getAcceleration( controller.state, controller.markers[ markerIndex ].getBody(), controller.markers[ markerIndex ].getOffset(), markerAcceleration );
    
    axisMeasuresList[ markerIndex ]->position = markerPosition[ controller.markerReferenceAxes[ markerIndex ] ];
    axisMeasuresList[ markerIndex ]->velocity = markerVelocity[ controller.markerReferenceAxes[ markerIndex ] ];
    axisMeasuresList[ markerIndex ]->acceleration = markerAcceleration[ controller.markerReferenceAxes[ markerIndex ] ];
    //axisMeasuresTable[ markerIndex ]->force = jointMeasuresTable[ markerIndex ]->force;
    
    markerPosition[ controller.markerReferenceAxes[ markerIndex ] ] = axisSetpointsList[ markerIndex ]->position;
    //controller.markersReference.setValue( markerIndex, markerPosition );
	controller.markersReference._markerData->_frames.get( 0 )->updMarker( 0 ) = SimTK::Vec3( 0 );
  }
  
  controller.ikSolver->track( controller.state );
  
  for( size_t jointIndex = 0; jointIndex < controller.actuatorsList.size(); jointIndex++ )
  {
    size_t actuatorInputsIndex = jointIndex * EMG_INPUT_VARS_NUMBER;
    axisMeasuresList[ jointIndex ]->position = actuatorInputs[ actuatorInputsIndex + EMG_POSITION ];
    axisMeasuresList[ jointIndex ]->velocity = actuatorInputs[ actuatorInputsIndex + EMG_VELOCITY ];
    axisMeasuresList[ jointIndex ]->acceleration = actuatorInputs[ actuatorInputsIndex + EMG_ACCELERATION ];
    size_t actuatorOutputsIndex = jointIndex * EMG_OUTPUT_VARS_NUMBER;
    axisMeasuresList[ jointIndex ]->force = actuatorOutputs[ actuatorOutputsIndex + EMG_TORQUE_INT ];
    axisMeasuresList[ jointIndex ]->stiffness = actuatorOutputs[ actuatorOutputsIndex + EMG_STIFFNESS ];

    jointSetpointsList[ jointIndex ]->position = axisSetpointsList[ jointIndex ]->position;
    jointSetpointsList[ jointIndex ]->velocity = axisSetpointsList[ jointIndex ]->velocity;
    jointSetpointsList[ jointIndex ]->acceleration = axisSetpointsList[ jointIndex ]->acceleration;
    jointSetpointsList[ jointIndex ]->force = axisSetpointsList[ jointIndex ]->force;
  }

  //std::cout << "joint 0 position: " << controller.actuatorsList[ 0 ]->getCoordinate()->getValue( state ) << std::endl;
}
