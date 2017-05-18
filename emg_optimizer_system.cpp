#include "emg_optimizer_system.h"

int EMGOptimizerSystem::JOINT_VARIABLES_NUMBER = 5;

EMGOptimizerSystem::EMGOptimizerSystem( int parametersNumber, int samplesNumber, SimTK::State& state, OpenSim::Model& model ) 
: OptimizerSystem( parametersNumber ), internalState( state ), internalModel( model )
{
  maxSamplesCount = samplesNumber;
}

EMGOptimizerSystem::~EMGOptimizerSystem()
{
  samplesList.clear();
}

int EMGOptimizerSystem::objectiveFunc( SimTK::Vector& parameters, const bool new_coefficients, SimTK::Real& f ) const
{
  return 0;
}

void EMGOptimizerSystem::CalculateTorques( SimTK::Vector& inputs, SimTK::Vector& outputs )
{
  
}

bool EMGOptimizerSystem::StoreSample( SimTK::Vector& sample )
{
  if( samplesList.size() >= maxSamplesCount ) return false;
  
  samplesList.push_back( sample );
  
  return true;
}

void EMGOptimizerSystem::ResetSampleStorage()
{
  samplesList.clear();
}
