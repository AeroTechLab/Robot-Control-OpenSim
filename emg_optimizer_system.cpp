#include "emg_optimizer_system.h"

int EMGOptimizerSystem::POSITION_VARIABLES_NUMBER = 3;
int EMGOptimizerSystem::FORCE_VARIABLES_NUMBER = 2;

EMGOptimizerSystem::EMGOptimizerSystem( int parametersNumber, int samplesNumber, SimTK::State& state, OpenSim::Model& model ) 
: OptimizerSystem( parametersNumber ), internalState( state ), internalModel( model )
{
  maxSamplesCount = samplesNumber;
}

EMGOptimizerSystem::~EMGOptimizerSystem()
{
  ResetSamplesStorage();
}

int EMGOptimizerSystem::objectiveFunc( SimTK::Vector& parameters, const bool new_coefficients, SimTK::Real& f ) const
{
  return 0;
}

SimTK::Vector EMGOptimizerSystem::CalculateTorques( SimTK::State& state, SimTK::Vector& emgInput, SimTK::Vector& positionsInput )
{
  SimtK::Vector forceOutputs( FORCE_VARIABLES_NUMBER /**jointsNumber*/ );
  
}

bool EMGOptimizerSystem::StoreSamples( SimTK::Vector& emgSample, SimTK::Vector& positionSample, SimTK::Vector& forceSample )
{
  if( emgSamplesList.size() >= maxSamplesCount ) return false;
  
  emgSamplesList.push_back( emgSample );
  positionSamplesList.push_back( positionSample );
  forceSamplesList.push_back( forceSample );
  
  return true;
}

void EMGOptimizerSystem::ResetSamplesStorage()
{
  emgSamplesList.clear();
  positionSamplesList.clear();
  forceSamplesList.clear();
}
