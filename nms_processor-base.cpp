#include "nms_processor-base.h"

NMSProcessorBase::NMSProcessorBase( const size_t parametersNumber, const size_t samplesNumber ) 
  : OptimizerSystem( parametersNumber ), MAX_SAMPLES_COUNT( samplesNumber ) { std::cout << "Parameters number: " << parametersNumber << std::endl; }
    
NMSProcessorBase::~NMSProcessorBase() { }

bool NMSProcessorBase::StoreSamples( SimTK::Vector& dynInputSample, SimTK::Vector& emgInputSample, SimTK::Vector& outputSample )
{
  if( inputSamplesList.size() >= MAX_SAMPLES_COUNT ) return false;
  
  SimTK::Vector inputSample( dynInputSample.size() + emgInputSample.size() );
  for( size_t valueIndex = 0; valueIndex < dynInputSample.size(); valueIndex++ )
    inputSample[ valueIndex ] = dynInputSample[ valueIndex ];
  for( size_t valueIndex = 0; valueIndex < dynInputSample.size(); valueIndex++ )
    inputSample[ dynInputSample.size() + valueIndex ] = emgInputSample[ valueIndex ];
  
  inputSamplesList.push_back( inputSample );
  outputSamplesList.push_back( outputSample );
  
  return true;
}

void NMSProcessorBase::ResetSamplesStorage()
{
  inputSamplesList.clear();
  outputSamplesList.clear();
}
