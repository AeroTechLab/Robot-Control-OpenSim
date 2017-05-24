#include "emg_optimizer_system.h"

const int EMGOptimizerSystem::POSITION_VARIABLES_NUMBER = 3;
const int EMGOptimizerSystem::FORCE_VARIABLES_NUMBER = 2;

EMGOptimizerSystem::EMGOptimizerSystem( int parametersNumber, int samplesNumber, SimTK::State& state, OpenSim::Model& model ) 
: OptimizerSystem( parametersNumber ), momentArmSolver( model ), internalState( state ), internalModel( model ), MAX_SAMPLES_COUNT( samplesNumber )
{
}

EMGOptimizerSystem::~EMGOptimizerSystem()
{
  ResetSamplesStorage();
}

int EMGOptimizerSystem::objectiveFunc( const SimTK::Vector& parameters, bool new_coefficients, SimTK::Real& f ) const
{
  return 0;
}

void EMGOptimizerSystem::CalculateTorques( SimTK::State& state, SimTK::Vector& emgInputs, SimTK::Vector& positionInputs, SimTK::Vector& forceOutputs )
{
}

bool EMGOptimizerSystem::StoreSamples( SimTK::Vector& emgSample, SimTK::Vector& positionSample, SimTK::Vector& forceSample )
{
  if( emgSamplesList.size() >= MAX_SAMPLES_COUNT ) return false;
  
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
