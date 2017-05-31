#ifndef EMG_OPTIMIZER_SYSTEM_H
#define EMG_OPTIMIZER_SYSTEM_H

#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>

typedef OpenSim::Set<OpenSim::CoordinateActuator> ActuatorSet;

enum { EMG_ANGLE, EMG_VELOCITY, EMG_ACCELERATION, EMG_SETPOINT, EMG_POS_VARS_NUMBER };
enum { EMG_TORQUE, EMG_STIFFNESS, EMG_FORCE_VARS_NUMBER };
enum { EMG_MAX_FORCE, EMG_FIBER_LENGTH, EMG_SLACK_LENGTH, EMG_PENNATION_ANGLE, EMG_ACTIVATION_FACTOR, EMG_OPT_VARS_NUMBER };

class EMGOptimizerSystem : public SimTK::OptimizerSystem
{
  public:
    /* Constructor class. Parameters accessed in objectiveFunc() class */
    EMGOptimizerSystem( int, int, SimTK::State&, OpenSim::Model&, ActuatorSet& );
    ~EMGOptimizerSystem();
 
	int objectiveFunc( const SimTK::Vector&, bool, SimTK::Real& ) const;

    SimTK::Vector CalculateTorques( SimTK::State, SimTK::Vector ) const;
    bool StoreSamples( SimTK::Vector&, SimTK::Vector&, SimTK::Vector& );
    void ResetSamplesStorage();
    
  private:
    SimTK::State internalState;
    OpenSim::Model& internalModel;
    ActuatorSet& actuatorsSet;
    const OpenSim::InverseDynamicsSolver idSolver;
    const OpenSim::MomentArmSolver momentArmSolver;
    SimTK::Array_<SimTK::Vector> emgSamplesList, positionSamplesList, torqueSamplesList;
    const size_t MAX_SAMPLES_COUNT;
};

#endif // EMG_OPTIMIZER_SYSTEM_H
