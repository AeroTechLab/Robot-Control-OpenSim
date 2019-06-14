#ifndef NMS_PROCESSOR_H
#define NMS_PROCESSOR_H

#include "nms_processor-base.h"

class NMSProcessor : public NMSProcessorBase
{
  public:
    /* Constructor class. Parameters accessed in objectiveFunc() class */
    NMSProcessor( OpenSim::Model&, ActuatorsList&, const size_t );
    ~NMSProcessor();
 
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

#endif // NMS_PROCESSOR_H
