#ifndef EMG_OPTIMIZER_H
#define EMG_OPTIMIZER_H

#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>

typedef SimTK::Array_<OpenSim::CoordinateActuator*> ActuatorsList;

enum { EMG_POSITION, EMG_VELOCITY, EMG_ACCELERATION, EMG_SETPOINT, EMG_POS_VARS_NUMBER };
enum { EMG_TORQUE_EXT, EMG_TORQUE_INT, EMG_STIFFNESS, EMG_FORCE_VARS_NUMBER };

class EMGOptimizer : public SimTK::OptimizerSystem
{
  public:
    /* Constructor class. Parameters accessed in objectiveFunc() class */
    EMGOptimizer( const size_t parametersNumber, const size_t samplesNumber ) 
    : OptimizerSystem( parametersNumber ), MAX_SAMPLES_COUNT( samplesNumber ) {};
    virtual ~EMGOptimizer();
 
    virtual int objectiveFunc( const SimTK::Vector&, bool, SimTK::Real& ) const = 0;

    virtual SimTK::Vector CalculateTorques( SimTK::State&, SimTK::Vector ) const = 0;
    virtual bool StoreSamples( SimTK::Vector&, SimTK::Vector&, SimTK::Vector& ) = 0;
    virtual void ResetSamplesStorage() = 0;

    virtual SimTK::Vector GetInitialParameters() = 0;
    
  protected:
    const size_t MAX_SAMPLES_COUNT;
};

#endif // EMG_OPTIMIZER_H
