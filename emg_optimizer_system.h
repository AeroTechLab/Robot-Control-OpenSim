#ifndef EMG_OPTIMIZER_SYSTEM_H
#define EMG_OPTIMIZER_SYSTEM_H

#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>

typedef OpenSim::Set<OpenSim::CoordinateActuator> ActuatorSet;

enum class EMGPositionVariable : int { POSITION, VELOCITY, ACCELERATION, SETPOINT, VARS_NUMBER };
enum class EMGTorqueVariable : int { TORQUE, STIFFNESS, VARS_NUMBER };
enum class EMGOptimizationVariable : int { MAX_FORCE, FIBER_LENGTH, SLACK_LENGTH, PENNATION_ANGLE, ACTIVATION_FACTOR, VARS_NUMBER };

class EMGOptimizerSystem : public SimTK::OptimizerSystem
{
  public:
    /* Constructor class. Parameters accessed in objectiveFunc() class */
    EMGOptimizerSystem( int, int, SimTK::State&, OpenSim::Model&, ActuatorSet& );
    ~EMGOptimizerSystem();
 
    SimTK::Vector CalculateTorques( SimTK::State&, SimTK::Vector& );
    bool StoreSamples( SimTK::Vector&, SimTK::Vector&, SimTK::Vector& );
    void ResetSamplesStorage();
    
  private:
    SimTK::State internalState;
    OpenSim::Model& internalModel;
    ActuatorSet& actuatorsSet;
    OpenSim::InverseDynamicsSolver idSolver;
    OpenSim::MomentArmSolver momentArmSolver;
    SimTK::Array_<SimTK::Vector> emgSamplesList, positionSamplesList, torqueSamplesList;
    const size_t MAX_SAMPLES_COUNT;
};

#endif // EMG_OPTIMIZER_SYSTEM_H
