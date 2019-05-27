#include "emg_optimizer-osim.h"

#include <cmath>

enum { EMG_MAX_FORCE, EMG_FIBER_LENGTH, EMG_SLACK_LENGTH, EMG_PENNATION_ANGLE, EMG_ACTIVATION_FACTOR, EMG_OPT_VARS_NUMBER };

EMGOptimizerImpl::EMGOptimizerImpl( OpenSim::Model& model, ActuatorsList& actuatorsList, const size_t samplesNumber ) 
: EMGOptimizer( EMG_OPT_VARS_NUMBER * model.getMuscles().getSize(), samplesNumber ), internalModel( model ), actuatorsList( actuatorsList )
{
  activationFactorsList.resize( internalModel.getMuscles().getSize() );
  
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
  SimTK::Vector initialParametersList( EMG_OPT_VARS_NUMBER * internalModel.getMuscles().getSize() );
  OpenSim::Set<OpenSim::Muscle> muscleSet = internalModel.getMuscles();
  for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
  {
    int parametersIndex = muscleIndex * EMG_OPT_VARS_NUMBER;
    initialParametersList[ parametersIndex + EMG_MAX_FORCE ] = muscleSet[ muscleIndex ].get_max_isometric_force();
    initialParametersList[ parametersIndex + EMG_FIBER_LENGTH ] = muscleSet[ muscleIndex ].get_optimal_fiber_length();
    initialParametersList[ parametersIndex + EMG_SLACK_LENGTH ] = muscleSet[ muscleIndex ].get_tendon_slack_length();
    initialParametersList[ parametersIndex + EMG_PENNATION_ANGLE ] = muscleSet[ muscleIndex ].get_pennation_angle_at_optimal();
    initialParametersList[ parametersIndex + EMG_ACTIVATION_FACTOR ] = -2.0;
  }

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
  for( size_t sampleIndex = 0; sampleIndex < inputSamplesList.size(); sampleIndex++ )
  {
    SimTK::Vector inputSample = inputSamplesList[ sampleIndex ];
    SimTK::Vector emgSample( activationFactorsList.size() );
    for( size_t valueIndex = 0; valueIndex < emgSample.size(); valueIndex++ )
        emgSample[ valueIndex ] = inputSample[ valueIndex ];
    SimTK::Vector extraInputSample( EMG_INPUT_VARS_NUMBER * actuatorsList.size() );
    for( size_t valueIndex = 0; valueIndex < extraInputSample.size(); valueIndex++ )
        extraInputSample[ valueIndex ] = inputSample[ emgSample.size() + valueIndex ];
    SimTK::Vector outputSample = outputSamplesList[ sampleIndex ];

    SimTK::Vector calculatedOutputs = CalculateOutputs( emgSample, extraInputSample );
    
    for( size_t jointIndex = 0; jointIndex < actuatorsList.size(); jointIndex++ )
    {
      int torqueOutputIndex = jointIndex * EMG_OUTPUT_VARS_NUMBER + EMG_TORQUE_INT;
      remainingError += std::pow( outputSample[ torqueOutputIndex ] - calculatedOutputs[ torqueOutputIndex ], 2.0 );
      int stiffnessOutputIndex = jointIndex * EMG_OUTPUT_VARS_NUMBER + EMG_STIFFNESS;
      remainingError += std::pow( outputSample[ stiffnessOutputIndex ] - calculatedOutputs[ stiffnessOutputIndex ], 2.0 );
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
  SimTK::State& state = internalModel.initSystem();
  SimTK::Vector torqueInternalOutputs( EMG_OUTPUT_VARS_NUMBER * actuatorsList.size() );

  try
  {
    for( size_t jointIndex = 0; jointIndex < actuatorsList.size(); jointIndex++ )
    {
      OpenSim::Coordinate* jointCoordinate = actuatorsList[ jointIndex ]->getCoordinate();
      int extraInputsIndex = jointIndex * EMG_INPUT_VARS_NUMBER;
      jointCoordinate->setValue( state, extraInputSample[ extraInputsIndex + EMG_POSITION ] );
      jointCoordinate->setSpeedValue( state, extraInputSample[ extraInputsIndex + EMG_VELOCITY ] );
#ifdef OSIM_LEGACY
      actuatorsList[ jointIndex ]->setOverrideForce( state, extraInputSample[ extraInputsIndex + EMG_TORQUE_EXT ] );
#else
      actuatorsList[ jointIndex ]->setOverrideActuation( state, extraInputSample[ extraInputsIndex + EMG_TORQUE_EXT ] );
#endif
    }
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
      int torqueIndex = jointIndex * EMG_OUTPUT_VARS_NUMBER + EMG_TORQUE_INT;
      int stiffnessIndex = jointIndex * EMG_OUTPUT_VARS_NUMBER + EMG_STIFFNESS;
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
