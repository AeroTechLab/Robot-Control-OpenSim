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

        OpenSim::ForceSet &forceSet = controller.osimModel->updForceSet();
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


void PreProcessSample( SimTK::Vector& positionSample, SimTK::Vector& torqueSample )
{
  OpenSim::InverseDynamicsSolver idSolver( *(controller.osimModel) );

  SimTK::Vector accelerationsList( controller.osimModel->getCoordinateSet().getSize(), 0.0 );
  for( size_t jointIndex = 0; jointIndex < controller.actuatorsList.size(); jointIndex++ )
  {
    OpenSim::Coordinate* jointCoordinate = controller.actuatorsList[ jointIndex ]->getCoordinate();
    int positionOutputsIndex = jointIndex * EMG_POS_VARS_NUMBER;
    jointCoordinate->setValue( controller.state, positionSample[ positionOutputsIndex + EMG_POSITION ] );
    jointCoordinate->setSpeedValue( controller.state, positionSample[ positionOutputsIndex + EMG_VELOCITY ] );
    int jointAccelerationIndex = controller.accelerationIndexesList[ jointIndex ];
    accelerationsList[ jointAccelerationIndex ] = positionSample[ positionOutputsIndex + EMG_ACCELERATION ];
    int torqueOutputIndex = jointIndex * EMG_FORCE_VARS_NUMBER;
#ifdef OSIM_LEGACY
    controller.actuatorsList[ jointIndex ]->setOverrideForce( controller.state, torqueSample[ torqueOutputIndex + EMG_TORQUE_EXT ] );
#else
    controller.actuatorsList[ jointIndex ]->setOverrideActuation( controller.state, torqueSample[ torqueOutputIndex + EMG_TORQUE_EXT ] );
#endif
  }
  
  try
  {
    SimTK::Vector idForcesList = idSolver.solve( controller.state, accelerationsList );
    
    for( int jointIndex = 0; jointIndex < controller.actuatorsList.size(); jointIndex++ )
    {
      int positionOutputsIndex = jointIndex * EMG_POS_VARS_NUMBER;
      double positionError = positionSample[ positionOutputsIndex + EMG_SETPOINT ] - positionSample[ positionOutputsIndex + EMG_POSITION ];
      int jointTorqueIndex = controller.accelerationIndexesList[ jointIndex ];
      std::cout << "joint " << jointIndex << " coordinate index: " << jointTorqueIndex << std::endl;
      int torqueOutputIndex = jointIndex * EMG_FORCE_VARS_NUMBER;
      torqueSample[ torqueOutputIndex + EMG_TORQUE_INT ] = idForcesList[ jointIndex ];
      torqueSample[ torqueOutputIndex + EMG_STIFFNESS ] = ( std::abs( positionError ) > 1.0e-6 ) ? idForcesList[ jointIndex ] / ( positionError ) : 100.0;
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

  SimTK::Vector positionOutputs( EMG_POS_VARS_NUMBER * controller.actuatorsList.size() );
  SimTK::Vector torqueOutputs( EMG_FORCE_VARS_NUMBER * controller.actuatorsList.size() );
  for( size_t jointIndex = 0; jointIndex < controller.actuatorsList.size(); jointIndex++ )
  {
    OpenSim::Coordinate* jointCoordinate = controller.actuatorsList[ jointIndex ]->getCoordinate();
    jointCoordinate->setValue( controller.state, jointMeasuresList[ jointIndex ]->position );
    jointCoordinate->setSpeedValue( controller.state, jointMeasuresList[ jointIndex ]->velocity );
#ifdef OSIM_LEGACY
    controller.actuatorsList[ jointIndex ]->setForce( controller.state, jointMeasuresList[ jointIndex ]->force );
#else
    controller.actuatorsList[ jointIndex ]->setActuation( controller.state, jointMeasuresList[ jointIndex ]->force );
#endif
    size_t positionOutputsIndex = jointIndex * EMG_POS_VARS_NUMBER;
    positionOutputs[ positionOutputsIndex + EMG_POSITION ] = jointMeasuresList[ jointIndex ]->position;
    positionOutputs[ positionOutputsIndex + EMG_VELOCITY ] = jointMeasuresList[ jointIndex ]->velocity;
    positionOutputs[ positionOutputsIndex + EMG_ACCELERATION ] = jointMeasuresList[ jointIndex ]->acceleration;
    positionOutputs[ positionOutputsIndex + EMG_SETPOINT ] = jointMeasuresList[ jointIndex ]->acceleration;
    size_t torqueOutputIndex = jointIndex * EMG_FORCE_VARS_NUMBER;
    torqueOutputs[ torqueOutputIndex + EMG_TORQUE_EXT ] = jointMeasuresList[ jointIndex ]->force;
  }
  
  PreProcessSample( positionOutputs, torqueOutputs );
  
  if( controller.currentControlState == ROBOT_PREPROCESSING )
    controller.emgProcessor->StoreSamples( controller.emgInputs, positionOutputs, torqueOutputs );
  else if( controller.currentControlState == ROBOT_OPERATION )
    torqueOutputs = controller.emgProcessor->CalculateTorques( controller.state, controller.emgInputs );
  
  OpenSim::Manager manager( *(controller.osimModel) );
#ifdef OSIM_LEGACY
  manager.integrate( controller.state, timeDelta );
#else
  controller.state = manager.integrate( timeDelta );
#endif
  
  for( size_t axisIndex = 0; axisIndex < controller.actuatorsList.size(); axisIndex++ )
  {
    OpenSim::Coordinate* axisCoordinate = controller.actuatorsList[ axisIndex ]->getCoordinate();
    axisMeasuresList[ axisIndex ]->position = axisCoordinate->getValue( controller.state );
    axisMeasuresList[ axisIndex ]->velocity = axisCoordinate->getSpeedValue( controller.state );
    axisMeasuresList[ axisIndex ]->acceleration = axisCoordinate->getAccelerationValue( controller.state );
    size_t torqueOutputIndex = axisIndex * EMG_FORCE_VARS_NUMBER;
    axisMeasuresList[ axisIndex ]->force = torqueOutputs[ torqueOutputIndex + EMG_TORQUE_EXT ];
    axisMeasuresList[ axisIndex ]->stiffness = torqueOutputs[ torqueOutputIndex + EMG_STIFFNESS ];
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
