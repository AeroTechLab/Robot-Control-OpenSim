#ifndef EMG_OPTIMIZER_SYSTEM_H
#define EMG_OPTIMIZER_SYSTEM_H

#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>

class EMGOptimizerSystem : public SimTK::OptimizerSystem
{
  public:
    /* Constructor class. Parameters accessed in objectiveFunc() class */
    EMGOptimizerSystem( int, int, SimTK::State&, OpenSim::Model& );
    ~EMGOptimizerSystem();
 
    SimTK::Vector CalculateTorques( SimTK::State&, SimTK::Vector&, SimTK::Vector& );
    bool StoreSample( SimTK::Vector& );
    void ResetSampleStorage();
    
    static const int JOINT_VARIABLES_NUMBER;
    
  private:
    SimTK::State internalState;
    OpenSim::Model& internalModel;
    SimTK::Array_<SimTK::Vector> samplesList;
    int maxSamplesCount;
}

#endif // EMG_OPTIMIZER_SYSTEM_H
