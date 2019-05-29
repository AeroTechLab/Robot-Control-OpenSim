#ifndef EMG_OPTIMIZER_IMPL_H
#define EMG_OPTIMIZER_IMPL_H

#include "emg_optimizer.h"

class EMGOptimizerImpl : public EMGOptimizer
{
  public:
    /* Constructor class. Parameters accessed in objectiveFunc() class */
    EMGOptimizerImpl( OpenSim::Model&, ActuatorsList&, const size_t );
    ~EMGOptimizerImpl();
 
    int objectiveFunc( const SimTK::Vector&, bool, SimTK::Real& ) const;

    SimTK::Vector CalculateOutputs( const SimTK::Vector&, const SimTK::Vector& ) const;

    SimTK::Vector GetInitialParameters();
    void SetParameters( const SimTK::Vector& );
    
  private:
    OpenSim::Model& internalModel;
    ActuatorsList& actuatorsList;
    SimTK::Vector activationFactorsList;

    //Log optimizationLog;
};

#endif // EMG_OPTIMIZER_IMPL_H
