#include "emg_optimizer_system.h"

#include <cmath>

EMGOptimizerSystem::EMGOptimizerSystem( int parametersNumber, int samplesNumber, SimTK::State& state, OpenSim::Model& model, ActuatorSet& actuatorsList ) 
: OptimizerSystem( parametersNumber ), 
  internalState( state ), internalModel( model ), actuatorsSet( actuatorsList ), idSolver( model ), momentArmSolver( model ), MAX_SAMPLES_COUNT( samplesNumber )
{
  internalState.updTime() = 0.0;
}

EMGOptimizerSystem::~EMGOptimizerSystem()
{
  ResetSamplesStorage();
}

int EMGOptimizerSystem::objectiveFunc( const SimTK::Vector& parametersList, bool newCoefficients, SimTK::Real& remainingError ) const
{
  OpenSim::Set<OpenSim::Muscle>& muscleSet = internalModel.updMuscles();
  SimTK::Vector activationFactorsList( muscleSet.getSize() );
  for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
  {
    int parametersIndex = muscleIndex * EMG_OPT_VARS_NUMBER;
    muscleSet[ muscleIndex ].set_max_isometric_force( parametersList[ parametersIndex + EMG_MAX_FORCE ] );
    muscleSet[ muscleIndex ].set_optimal_fiber_length( parametersList[ parametersIndex + EMG_FIBER_LENGTH ] );
    muscleSet[ muscleIndex ].set_tendon_slack_length( parametersList[ parametersIndex + EMG_SLACK_LENGTH ] );
    muscleSet[ muscleIndex ].set_pennation_angle_at_optimal( parametersList[ parametersIndex + EMG_PENNATION_ANGLE ] );
    //muscleSet[ muscleIndex ].set_max_contraction_velocity( parametersList[ 4 ] );
    activationFactorsList[ muscleIndex ] = parametersList[ parametersIndex + EMG_ACTIVATION_FACTOR ];
  }
  
  for( size_t sampleIndex = 0; sampleIndex < emgSamplesList.size(); sampleIndex++ )
  {
	SimTK::Vector emgSample = emgSamplesList[ sampleIndex ];
    SimTK::Vector positionSample = positionSamplesList[ sampleIndex ];
	SimTK::Vector torqueSample = torqueSamplesList[ sampleIndex ];
    
    for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
      muscleSet[ muscleIndex ].setDisabled( (const SimTK::State) internalState, true ); // setAppliesForce -> false
    
    SimTK::Vector accelerationsList( actuatorsSet.getSize(), 0.0 );
    for( int jointIndex = 0; jointIndex < actuatorsSet.getSize(); jointIndex++ )
    {
      OpenSim::Coordinate* jointCoordinate = actuatorsSet[ jointIndex ].getCoordinate();
      int positionInputsIndex = jointIndex * EMG_POS_VARS_NUMBER;
      jointCoordinate->setValue( (const SimTK::State) internalState, positionSample[ positionInputsIndex + EMG_ANGLE ] );
      jointCoordinate->setSpeedValue( (const SimTK::State) internalState, positionSample[ positionInputsIndex + EMG_VELOCITY ] );
      accelerationsList[ jointIndex ] = positionSample[ positionInputsIndex + EMG_ACCELERATION ];
      int torqueOutputsIndex = jointIndex * EMG_FORCE_VARS_NUMBER;
      actuatorsSet[ jointIndex ].setForce( internalState, torqueSample[ torqueOutputsIndex + EMG_TORQUE ] );
    }
    
    internalModel.getMultibodySystem().realize( internalState, SimTK::Stage::Position );
    internalModel.getMultibodySystem().realize( internalState, SimTK::Stage::Velocity );
    internalModel.getMultibodySystem().realize( internalState, SimTK::Stage::Acceleration );
    
	SimTK::Vector idForcesList = const_cast<OpenSim::InverseDynamicsSolver&>(idSolver).solve( internalState, accelerationsList );
    
    for( int jointIndex = 0; jointIndex < actuatorsSet.getSize(); jointIndex++ )
    {
      int positionInputsIndex = jointIndex * EMG_POS_VARS_NUMBER;
      double position = positionSample[ positionInputsIndex + EMG_ANGLE ];
      double setpoint = positionSample[ positionInputsIndex + EMG_SETPOINT ];
      int torqueOutputsIndex = jointIndex * EMG_FORCE_VARS_NUMBER;
      torqueSample[ torqueOutputsIndex + EMG_TORQUE ] = idForcesList[ jointIndex ];
      torqueSample[ torqueOutputsIndex + EMG_STIFFNESS ] = ( setpoint - position ) / idForcesList[ jointIndex ];
    }
    
    for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
      muscleSet[ muscleIndex ].setDisabled( (const SimTK::State) internalState, false ); // setAppliesForce -> true
    
	SimTK::Vector emgTorqueOutputs = CalculateTorques( internalState, emgSample );
    
    remainingError = 0.0;
    for( int jointIndex = 0; jointIndex < actuatorsSet.getSize(); jointIndex++ )
    {
      int torqueOutputIndex = jointIndex * EMG_FORCE_VARS_NUMBER + EMG_TORQUE;
      remainingError += std::pow( torqueSample[ torqueOutputIndex ] - emgTorqueOutputs[ torqueOutputIndex ], 2.0 );
      int stiffnessOutputIndex = jointIndex * EMG_FORCE_VARS_NUMBER + EMG_STIFFNESS;
      remainingError += std::pow( torqueSample[ stiffnessOutputIndex ] - emgTorqueOutputs[ stiffnessOutputIndex ], 2.0 );
    }
  }
  
  return 0;
}

SimTK::Vector EMGOptimizerSystem::CalculateTorques( SimTK::State state, SimTK::Vector emgInputs ) const
{
  OpenSim::Set<OpenSim::Muscle> muscleSet = internalModel.getMuscles();
  SimTK::Vector muscleForcesList( muscleSet.getSize() );
  for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
  {
    muscleSet[ muscleIndex ].setActivation( state, emgInputs[ muscleIndex ] );
    muscleForcesList[ muscleIndex ] = muscleSet[ muscleIndex ].getActiveFiberForce( state ) + muscleSet[ muscleIndex ].getPassiveFiberForce( state );
  }
  
  SimTK::Vector torqueOutputs( EMG_FORCE_VARS_NUMBER * actuatorsSet.getSize() );
  for( int jointIndex = 0; jointIndex < actuatorsSet.getSize(); jointIndex++ )
  {
    OpenSim::Coordinate* jointCoordinate = actuatorsSet[ jointIndex ].getCoordinate();
    int torqueOutputIndex = jointIndex * EMG_FORCE_VARS_NUMBER + EMG_TORQUE;
    int stiffnessOutputIndex = jointIndex * EMG_FORCE_VARS_NUMBER + EMG_STIFFNESS;
    torqueOutputs[ torqueOutputIndex ] = torqueOutputs[ stiffnessOutputIndex ] = 0.0;
    for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
    {
      const OpenSim::GeometryPath& muscleGeometry = muscleSet[ muscleIndex ].get_GeometryPath();
      double muscleJointMomentArm = momentArmSolver.solve( state, *jointCoordinate, muscleGeometry );
      double muscleJointTorque = muscleForcesList[ muscleIndex ] * muscleJointMomentArm;
      torqueOutputs[ torqueOutputIndex ] += muscleJointTorque;
      torqueOutputs[ stiffnessOutputIndex ] += std::abs( muscleJointTorque );
    }
  }
  
  return torqueOutputs;
}

bool EMGOptimizerSystem::StoreSamples( SimTK::Vector& emgSample, SimTK::Vector& positionSample, SimTK::Vector& torqueSample )
{
  if( emgSamplesList.size() >= MAX_SAMPLES_COUNT ) return false;
  
  emgSamplesList.push_back( emgSample );
  positionSamplesList.push_back( positionSample );
  torqueSamplesList.push_back( torqueSample );
  
  return true;
}

void EMGOptimizerSystem::ResetSamplesStorage()
{
  emgSamplesList.clear();
  positionSamplesList.clear();
  torqueSamplesList.clear();
}
