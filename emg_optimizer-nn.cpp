#include "emg_optimizer-nn.h"

#include "perceptron/multi_layer_perceptron.h"

EMGOptimizerImpl::EMGOptimizerImpl( OpenSim::Model& model, ActuatorsList& actuatorsList, const size_t samplesNumber ) 
: EMGOptimizer( 2, samplesNumber )
{
  inputsNumber = model.getMuscles().getSize() + EMG_INPUT_VARS_NUMBER * actuatorsList.size();
  outputsNumber = EMG_OUTPUT_VARS_NUMBER * actuatorsList.size();
  
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

    SimTK::Vector emgTorqueOutputs = CalculateOutputs( state, emgSample );
    
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

SimTK::Vector EMGOptimizerImpl::CalculateOutputs( const SimTK::Vector& emgInputs, const SimTK::Vector& extraInputSample ) const
{
  double* inputsList = new double[ inputsNumber ];
  double* outputsList = new double[ outputsNumber ];
  
  for( size_t valueIndex = 0; valueIndex < emgInputs.size(); valueIndex++ )
    inputsList[ valueIndex ] = emgInputs[ valueIndex ];
  for( size_t valueIndex = 0; valueIndex < extraInputSample.size(); valueIndex++ )
    inputsList[ emgInputs.size() + valueIndex ] = extraInputSample[ valueIndex ];
  
  MLPerceptron_ProcessInput( perceptron, inputsList, outputsList );
  
  SimTK::Vector torqueInternalOutputs( outputsNumber, outputsList );
  
  delete[] outputsList;
  
  return torqueInternalOutputs;
}
