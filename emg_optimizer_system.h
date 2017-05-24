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
 
    void CalculateTorques( SimTK::State&, SimTK::Vector&, SimTK::Vector&, SimTK::Vector& );
    bool StoreSamples( SimTK::Vector&, SimTK::Vector&, SimTK::Vector& );
    void ResetSamplesStorage();
    
	virtual int EMGOptimizerSystem::objectiveFunc( const SimTK::Vector&, bool, SimTK::Real& ) const;

    static const int POSITION_VARIABLES_NUMBER, FORCE_VARIABLES_NUMBER;
    
  private:
    SimTK::State internalState;
    OpenSim::Model& internalModel;
	OpenSim::MomentArmSolver momentArmSolver;
    SimTK::Array_<SimTK::Vector> emgSamplesList, positionSamplesList, forceSamplesList;
    const size_t MAX_SAMPLES_COUNT;
};

#endif // EMG_OPTIMIZER_SYSTEM_H
