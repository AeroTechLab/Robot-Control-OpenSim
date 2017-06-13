#include "emg_optimizer_system.h"

#include <OpenSim/Simulation/InverseDynamicsSolver.h>

#include <cmath>

EMGOptimizerSystem::EMGOptimizerSystem( OpenSim::Model& model, ActuatorsList& actuatorsList, int samplesNumber ) 
: OptimizerSystem( EMG_OPT_VARS_NUMBER * model.getMuscles().getSize() ), internalModel( model ), actuatorsList( actuatorsList ), momentArmSolver( model ), MAX_SAMPLES_COUNT( samplesNumber )
{
  activationFactorsList.resize( internalModel.getMuscles().getSize() );

  SimTK::Vector initialParametersList = GetInitialParameters();
  SimTK::Vector parametersMinList( initialParametersList.size() ), parametersMaxList( initialParametersList.size() );
  for( int parameterIndex = 0; parameterIndex < initialParametersList.size(); parameterIndex++ )
  {
	parametersMinList[ parameterIndex ] = 0.5 * initialParametersList[ parameterIndex ];
	parametersMaxList[ parameterIndex ] = 1.5 * initialParametersList[ parameterIndex ];
  }
  //setParameterLimits( parametersMinList, parametersMaxList );

  DataLogging.SetBaseDirectory( "test" );
  optimizationLog = DataLogging.InitLog( "joints/optimization", 6 );
}

EMGOptimizerSystem::~EMGOptimizerSystem()
{
  ResetSamplesStorage();

  DataLogging.EndLog( optimizationLog );
}

void EMGOptimizerSystem::PreProcessSamples()
{
  SimTK::State& state = internalModel.initSystem();
  OpenSim::InverseDynamicsSolver idSolver( internalModel );
  
  OpenSim::Set<OpenSim::Muscle>& muscleSet = internalModel.updMuscles();
  for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
    muscleSet[ muscleIndex ].setDisabled( state, true ); // setAppliesForce -> false

  for( size_t sampleIndex = 0; sampleIndex < torqueSamplesList.size(); sampleIndex++ )
  {
	SimTK::Vector& positionSample = positionSamplesList[ sampleIndex ];
	SimTK::Vector& torqueSample = torqueSamplesList[ sampleIndex ];

	SimTK::Vector accelerationsList( internalModel.getCoordinateSet().getSize(), 0.0 );
    for( size_t jointIndex = 0; jointIndex < actuatorsList.size(); jointIndex++ )
    {
      OpenSim::Coordinate* jointCoordinate = actuatorsList[ jointIndex ]->getCoordinate();
      int positionInputsIndex = jointIndex * EMG_POS_VARS_NUMBER;
      jointCoordinate->setValue( state, positionSample[ positionInputsIndex + EMG_ANGLE ] );
      jointCoordinate->setSpeedValue( state, positionSample[ positionInputsIndex + EMG_VELOCITY ] );
      int jointAccelerationIndex = internalModel.getCoordinateSet().getIndex( jointCoordinate );
	  accelerationsList[ jointAccelerationIndex ] = positionSample[ positionInputsIndex + EMG_ACCELERATION ];
      int torqueOutputsIndex = jointIndex * EMG_FORCE_VARS_NUMBER;
      actuatorsList[ jointIndex ]->setForce( state, torqueSample[ torqueOutputsIndex + EMG_TORQUE_EXT ] );
    }
    
	try
	{
	  SimTK::Vector idForcesList = idSolver.solve( state, accelerationsList );

	  for( int jointIndex = 0; jointIndex < accelerationsList.size(); jointIndex++ )
      {
        int positionInputsIndex = jointIndex * EMG_POS_VARS_NUMBER;
        double position = positionSample[ positionInputsIndex + EMG_ANGLE ];
        double setpoint = positionSample[ positionInputsIndex + EMG_SETPOINT ];
		int jointTorqueIndex = internalModel.getCoordinateSet().getIndex( actuatorsList[ jointIndex ]->getCoordinate() );
		std::cout << "joint " << jointIndex << " coordinate index: " << jointTorqueIndex << std::endl;
        int torqueOutputsIndex = jointIndex * EMG_FORCE_VARS_NUMBER;
        torqueSample[ torqueOutputsIndex + EMG_TORQUE_INT ] = idForcesList[ jointIndex ];
        torqueSample[ torqueOutputsIndex + EMG_STIFFNESS ] = ( std::abs( setpoint - position ) > 1.0e-6 ) ? idForcesList[ jointIndex ] / ( setpoint - position ) : 100.0;
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

  for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
      muscleSet[ muscleIndex ].setDisabled( state, false ); // setAppliesForce -> true
}

SimTK::Vector EMGOptimizerSystem::GetInitialParameters()
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

int EMGOptimizerSystem::objectiveFunc( const SimTK::Vector& parametersList, bool newCoefficients, SimTK::Real& remainingError ) const
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
	for( int sampleIndex = 0; sampleIndex < EMG_POS_VARS_NUMBER; sampleIndex++ )
	  DataLogging.RegisterValues( optimizationLog, 1, positionSample[ sampleIndex ] );
	DataLogging.RegisterValues( optimizationLog, 1, torqueSample[ EMG_FORCE_VARS_NUMBER + EMG_TORQUE_EXT ] );
	for( int sampleIndex = 0; sampleIndex < emgSample.size(); sampleIndex++ )
	  DataLogging.RegisterValues( optimizationLog, 1, emgSample[ sampleIndex ] );
	DataLogging.RegisterValues( optimizationLog, 2, torqueSample[ EMG_FORCE_VARS_NUMBER + EMG_TORQUE_EXT ], emgTorqueOutputs[ EMG_FORCE_VARS_NUMBER + EMG_TORQUE_INT ] );
	DataLogging.RegisterValues( optimizationLog, 2, torqueSample[ EMG_FORCE_VARS_NUMBER + EMG_TORQUE_EXT ], emgTorqueOutputs[ EMG_FORCE_VARS_NUMBER + EMG_STIFFNESS ] );
	DataLogging.EnterNewLine( optimizationLog );
  }

  std::cout << "objective function error: " << remainingError << std::endl;

  return 0;
}

SimTK::Vector EMGOptimizerSystem::CalculateTorques( SimTK::State& state, SimTK::Vector emgInputs ) const
{
  SimTK::Vector torqueOutputs( EMG_FORCE_VARS_NUMBER * actuatorsList.size() );

  try
  {
    OpenSim::Set<OpenSim::Muscle>& muscleSet = internalModel.updMuscles();
    SimTK::Vector muscleForcesList( muscleSet.getSize() );
    for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
    {
	  double activation = ( std::exp( activationFactorsList[ muscleIndex ] * emgInputs[ muscleIndex ] ) - 1 ) / ( std::exp( activationFactorsList[ muscleIndex ] ) - 1 );
      muscleSet[ muscleIndex ].setActivation( state, emgInputs[ muscleIndex ] );
    }

    internalModel.equilibrateMuscles( state );

    for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
	  muscleForcesList[ muscleIndex ] = muscleSet[ muscleIndex ].getActiveFiberForce( state ) + muscleSet[ muscleIndex ].getPassiveFiberForce( state );
  
    for( size_t jointIndex = 0; jointIndex < actuatorsList.size(); jointIndex++ )
    {
      OpenSim::Coordinate* jointCoordinate = actuatorsList[ jointIndex ]->getCoordinate();
      int torqueOutputIndex = jointIndex * EMG_FORCE_VARS_NUMBER + EMG_TORQUE_EXT;
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
	  //std::cout << "joint " << jointIndex << " torque: " << torqueOutputs[ torqueOutputIndex ] << std::endl;
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
