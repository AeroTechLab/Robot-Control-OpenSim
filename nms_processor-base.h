#ifndef NMS_PROCESSOR_BASE_H
#define NMS_PROCESSOR_BASE_H

#include <OpenSim/OpenSim.h>

typedef std::vector<OpenSim::CoordinateActuator*> ActuatorsList;

enum { NMS_POSITION, NMS_VELOCITY, NMS_ACCELERATION, NMS_SETPOINT, NMS_TORQUE_EXT, NMS_INPUT_VARS_NUMBER };
enum { NMS_TORQUE_INT, NMS_STIFFNESS, NMS_OUTPUT_VARS_NUMBER };

class NMSProcessorBase : public SimTK::OptimizerSystem
{
  public:
    /* Constructor class. Parameters accessed in objectiveFunc() class */
    NMSProcessorBase( const size_t parametersNumber, const size_t samplesNumber );
    virtual ~NMSProcessorBase();
 
    virtual int objectiveFunc( const SimTK::Vector&, bool, SimTK::Real& ) const = 0;

    virtual SimTK::Vector CalculateOutputs( const SimTK::Vector&, const SimTK::Vector& ) const = 0;
    
    bool StoreSamples( SimTK::Vector&, SimTK::Vector&, SimTK::Vector& );
    
    void ResetSamplesStorage();

    virtual SimTK::Vector GetInitialParameters() = 0;
    
    virtual void SetParameters( const SimTK::Vector& ) = 0;
    
  protected:
    const size_t MAX_SAMPLES_COUNT;
    SimTK::Array_<SimTK::Vector> inputSamplesList, outputSamplesList;
};

#endif // NMS_PROCESSOR_BASE_H
