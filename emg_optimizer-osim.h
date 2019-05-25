#ifndef EMG_OPTIMIZER_IMPL_H
#define EMG_OPTIMIZER_IMPL_H

#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>

typedef SimTK::Array_<OpenSim::CoordinateActuator*> ActuatorsList;

enum { EMG_POSITION, EMG_VELOCITY, EMG_ACCELERATION, EMG_SETPOINT, EMG_POS_VARS_NUMBER };
enum { EMG_TORQUE_EXT, EMG_TORQUE_INT, EMG_STIFFNESS, EMG_FORCE_VARS_NUMBER };

class EMGOptimizerImpl : public EMGOptimizer
{
  public:
    /* Constructor class. Parameters accessed in objectiveFunc() class */
    EMGOptimizer( OpenSim::Model&, ActuatorsList&, int );
    ~EMGOptimizer();
 
    int objectiveFunc( const SimTK::Vector&, bool, SimTK::Real& ) const;

    SimTK::Vector CalculateTorques( SimTK::State&, SimTK::Vector ) const;
    bool StoreSamples( SimTK::Vector&, SimTK::Vector&, SimTK::Vector& );
    void ResetSamplesStorage();

    SimTK::Vector GetInitialParameters();
    
  private:
    OpenSim::Model& internalModel;
    ActuatorsList& actuatorsList;
    SimTK::Array_<SimTK::Vector> emgSamplesList, positionSamplesList, torqueSamplesList;
    SimTK::Vector activationFactorsList;
    const size_t MAX_SAMPLES_COUNT;

    //Log optimizationLog;
};

#endif // EMG_OPTIMIZER_IMPL_H
