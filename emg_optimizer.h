#ifndef EMG_OPTIMIZER_H
#define EMG_OPTIMIZER_H

#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>

typedef SimTK::Array_<OpenSim::CoordinateActuator*> ActuatorsList;

enum { EMG_POSITION, EMG_VELOCITY, EMG_ACCELERATION, EMG_SETPOINT, EMG_TORQUE_EXT, EMG_INPUT_VARS_NUMBER };
enum { EMG_TORQUE_INT, EMG_STIFFNESS, EMG_OUTPUT_VARS_NUMBER };

class EMGOptimizer : public SimTK::OptimizerSystem
{
  public:
    /* Constructor class. Parameters accessed in objectiveFunc() class */
    EMGOptimizer( const size_t parametersNumber, const size_t samplesNumber ) 
    : OptimizerSystem( parametersNumber ), MAX_SAMPLES_COUNT( samplesNumber ) {};
    virtual ~EMGOptimizer();
 
    virtual int objectiveFunc( const SimTK::Vector&, bool, SimTK::Real& ) const = 0;

    virtual SimTK::Vector CalculateOutputs( const SimTK::Vector&, const SimTK::Vector& ) const = 0;
    
    bool StoreSamples( SimTK::Vector& emgSample, SimTK::Vector& extraInputSample, SimTK::Vector& outputSample )
    {
      if( inputSamplesList.size() >= MAX_SAMPLES_COUNT ) return false;
      
      SimTK::Vector inputSample( emgSample.size() + extraInputSample.size() );
      for( size_t valueIndex = 0; valueIndex < emgSample.size(); valueIndex++ )
        inputSample[ valueIndex ] = emgSample[ valueIndex ];
      for( size_t valueIndex = 0; valueIndex < emgSample.size(); valueIndex++ )
        inputSample[ emgSample.size() + valueIndex ] = extraInputSample[ valueIndex ];
      
      inputSamplesList.push_back( inputSample );
      outputSamplesList.push_back( outputSample );
      
      return true;
    }
    
    void ResetSamplesStorage()
    {
      inputSamplesList.clear();
      outputSamplesList.clear();
    }

    virtual SimTK::Vector GetInitialParameters() = 0;
    
  protected:
    const size_t MAX_SAMPLES_COUNT;
    SimTK::Array_<SimTK::Vector> inputSamplesList, outputSamplesList;
};

#endif // EMG_OPTIMIZER_H
