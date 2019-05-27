#include "emg_optimizer-nn.h"

#include "perceptron/multi_layer_perceptron.h"

EMGOptimizerImpl::EMGOptimizerImpl( OpenSim::Model& model, ActuatorsList& actuatorsList, const size_t samplesNumber ) 
: EMGOptimizer( 2, samplesNumber )
{
  inputsNumber = model.getMuscles().getSize();
  outputsNumber = EMG_FORCE_VARS_NUMBER * actuatorsList.size();
  
  SimTK::Vector initialParametersList = GetInitialParameters();
  SimTK::Vector parametersMinList( initialParametersList.size() ), parametersMaxList( initialParametersList.size() );
  for( int parameterIndex = 0; parameterIndex < initialParametersList.size(); parameterIndex++ )
  {
    parametersMinList[ parameterIndex ] = 0.5 * initialParametersList[ parameterIndex ];
    parametersMaxList[ parameterIndex ] = 1.5 * initialParametersList[ parameterIndex ];
  }
  setParameterLimits( parametersMinList, parametersMaxList );

  //DataLogging.SetBaseDirectory( "test" );
  //optimizationLog = DataLogging.InitLog( "joints/optimization", 6 );
}

EMGOptimizerImpl::~EMGOptimizerImpl()
{
  ResetSamplesStorage();

  //DataLogging.EndLog( optimizationLog );
}

SimTK::Vector EMGOptimizerImpl::GetInitialParameters()
{
  SimTK::Vector initialParametersList( 2 );
  initialParametersList[ 0 ] = 10;
  initialParametersList[ 1 ] = MAX_SAMPLES_COUNT / 2;

  return initialParametersList;
}

int EMGOptimizerImpl::objectiveFunc( const SimTK::Vector& parametersList, bool newCoefficients, SimTK::Real& remainingError ) const
{
  SimTK::State& state = internalModel.initSystem();
  try
  {
    internalModel.equilibrateMuscles( state );
    OpenSim::Set<OpenSim::Muscle>& muscleSet = internalModel.updMuscles();
    for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
    {
      int parametersIndex = muscleIndex * EMG_OPT_VARS_NUMBER;
      muscleSet[ muscleIndex ].set_max_isometric_force( parametersList[ parametersIndex + EMG_MAX_FORCE ] );
      muscleSet[ muscleIndex ].set_optimal_fiber_length( parametersList[ parametersIndex + EMG_FIBER_LENGTH ] );
      muscleSet[ muscleIndex ].set_tendon_slack_length( parametersList[ parametersIndex + EMG_SLACK_LENGTH ] );
      //std::cout << "muscle " << muscleIndex << " pennation angle: " << parametersList[ parametersIndex + EMG_PENNATION_ANGLE ] << std::endl;
      //muscleSet[ muscleIndex ].set_pennation_angle_at_optimal( parametersList[ parametersIndex + EMG_PENNATION_ANGLE ] );
      const_cast<SimTK::Vector&>(activationFactorsList)[ muscleIndex ] = parametersList[ parametersIndex + EMG_ACTIVATION_FACTOR ];
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
  
  remainingError = 0.0;
  for( size_t sampleIndex = 0; sampleIndex < emgSamplesList.size(); sampleIndex++ )
  {
    SimTK::Vector emgSample = emgSamplesList[ sampleIndex ];
    SimTK::Vector positionSample = positionSamplesList[ sampleIndex ];
    SimTK::Vector torqueSample = torqueSamplesList[ sampleIndex ];

    SimTK::Vector emgTorqueOutputs = CalculateTorques( state, emgSample );
    
    for( size_t jointIndex = 0; jointIndex < actuatorsList.size(); jointIndex++ )
    {
      int torqueOutputIndex = jointIndex * EMG_FORCE_VARS_NUMBER + EMG_TORQUE_INT;
      remainingError += std::pow( torqueSample[ torqueOutputIndex ] - emgTorqueOutputs[ torqueOutputIndex ], 2.0 );
      int stiffnessOutputIndex = jointIndex * EMG_FORCE_VARS_NUMBER + EMG_STIFFNESS;
      remainingError += std::pow( torqueSample[ stiffnessOutputIndex ] - emgTorqueOutputs[ stiffnessOutputIndex ], 2.0 );
    }

	// Data logging for joint 0
// 	for( int sampleIndex = 0; sampleIndex < EMG_POS_VARS_NUMBER; sampleIndex++ )
// 	  DataLogging.RegisterValues( optimizationLog, 1, positionSample[ sampleIndex ] );
// 	DataLogging.RegisterValues( optimizationLog, 1, torqueSample[ EMG_FORCE_VARS_NUMBER + EMG_TORQUE_EXT ] );
// 	for( int sampleIndex = 0; sampleIndex < emgSample.size(); sampleIndex++ )
// 	  DataLogging.RegisterValues( optimizationLog, 1, emgSample[ sampleIndex ] );
// 	DataLogging.RegisterValues( optimizationLog, 2, torqueSample[ EMG_FORCE_VARS_NUMBER + EMG_TORQUE_EXT ], emgTorqueOutputs[ EMG_FORCE_VARS_NUMBER + EMG_TORQUE_INT ] );
// 	DataLogging.RegisterValues( optimizationLog, 2, torqueSample[ EMG_FORCE_VARS_NUMBER + EMG_TORQUE_EXT ], emgTorqueOutputs[ EMG_FORCE_VARS_NUMBER + EMG_STIFFNESS ] );
// 	DataLogging.EnterNewLine( optimizationLog );
  }

  std::cout << "objective function error: " << remainingError << std::endl;

  return 0;
}

SimTK::Vector EMGOptimizerImpl::CalculateTorques( SimTK::State& state, SimTK::Vector emgInputs ) const
{
  SimTK::Vector torqueInternalOutputs( EMG_FORCE_VARS_NUMBER * actuatorsList.size() );

  try
  {
    OpenSim::Set<OpenSim::Muscle>& muscleSet = internalModel.updMuscles();
    SimTK::Vector muscleForcesList( muscleSet.getSize() );
    for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
    {
      double activation = ( std::exp( activationFactorsList[ muscleIndex ] * emgInputs[ muscleIndex ] ) - 1 ) / ( std::exp( activationFactorsList[ muscleIndex ] ) - 1 );
#ifdef OSIM_LEGACY
      muscleSet[ muscleIndex ].setActivation( state, activation );
#else
      muscleSet[ muscleIndex ].setExcitation( state, activation );
#endif
    }

    internalModel.equilibrateMuscles( state );

    for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
      muscleForcesList[ muscleIndex ] = muscleSet[ muscleIndex ].getActiveFiberForce( state ) + muscleSet[ muscleIndex ].getPassiveFiberForce( state );
  
    for( size_t jointIndex = 0; jointIndex < actuatorsList.size(); jointIndex++ )
    {
      OpenSim::Coordinate* jointCoordinate = actuatorsList[ jointIndex ]->getCoordinate();
      int torqueIndex = jointIndex * EMG_FORCE_VARS_NUMBER + EMG_TORQUE_EXT;
      int stiffnessIndex = jointIndex * EMG_FORCE_VARS_NUMBER + EMG_STIFFNESS;
      torqueInternalOutputs[ torqueIndex ] = torqueInternalOutputs[ stiffnessIndex ] = 0.0;
      for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
      {
        double muscleJointMomentArm = muscleSet[ muscleIndex ].computeMomentArm( state, *jointCoordinate );
        double muscleJointTorque = muscleForcesList[ muscleIndex ] * muscleJointMomentArm;
        torqueInternalOutputs[ torqueIndex ] += muscleJointTorque;
        torqueInternalOutputs[ stiffnessIndex ] += std::abs( muscleJointTorque );
      }
      //std::cout << "joint " << jointIndex << " torque: " << torqueInternalOutputs[ torqueIndex ] << std::endl;
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
  
  return torqueInternalOutputs;
}

bool EMGOptimizerImpl::StoreSamples( SimTK::Vector& emgSample, SimTK::Vector& positionSample, SimTK::Vector& torqueSample )
{
  if( inputSamplesTable.size() >= MAX_SAMPLES_COUNT ) return false;
  
  inputSamplesTable.push_back( emgSample );
  outputSamplesTable.push_back( positionSample );
  
  return true;
}

void EMGOptimizerImpl::ResetSamplesStorage()
{
  inputSamplesTable.clear();
  outputSamplesTable.clear();
}
