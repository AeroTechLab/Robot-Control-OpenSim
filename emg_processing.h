#ifndef EMG_PROCESSING_H
#define EMG_PROCESSING_H

#include "namespaces.h"

#include "data_logging.h"

enum EMGProcessingPhase { EMG_PROCESSING_MEASUREMENT, EMG_PROCESSING_CALIBRATION, EMG_PROCESSING_OFFSET, EMG_PROCESSING_SAMPLING, EMG_PROCESSING_PHASES_NUMBER };

typedef struct _EMGJointData EMGJointData;
typedef EMGJointData* EMGJoint;

#define EMG_JOINT_MAX_MUSCLES_NUMBER 10
#define EMG_MAX_SAMPLES_NUMBER 5000


typedef struct _EMGJointSamplingData
{
  double muscleSignalsList[ EMG_JOINT_MAX_MUSCLES_NUMBER ][ EMG_MAX_SAMPLES_NUMBER ];
  double measuredAnglesList[ EMG_MAX_SAMPLES_NUMBER ];
  double setpointAnglesList[ EMG_MAX_SAMPLES_NUMBER ];
  double externalTorquesList[ EMG_MAX_SAMPLES_NUMBER ];
  double sampleTimesList[ EMG_MAX_SAMPLES_NUMBER ];
  size_t musclesCount, samplesCount;
  Log protocolLog;
  bool isLogging;
}
EMGJointSamplingData;

typedef EMGJointSamplingData* EMGJointSampler;


#define EMG_PROCESSING_FUNCTIONS( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( EMGJoint, Namespace, InitJoint, const char* ) \
        INIT_FUNCTION( void, Namespace, EndJoint, EMGJoint ) \
        INIT_FUNCTION( void, Namespace, GetJointMuscleSignals, EMGJoint, double* ) \
        INIT_FUNCTION( void, Namespace, GetJointMuscleTorques, EMGJoint, double*, double, double, double* ) \
        INIT_FUNCTION( double, Namespace, GetJointTorque, EMGJoint, double* ) \
        INIT_FUNCTION( double, Namespace, GetJointStiffness, EMGJoint, double* ) \
        INIT_FUNCTION( void, Namespace, SetJointProcessingPhase, EMGJoint, enum EMGProcessingPhase ) \
        INIT_FUNCTION( size_t, Namespace, GetJointMusclesCount, EMGJoint ) \
        INIT_FUNCTION( void, Namespace, FitJointParameters, EMGJoint, EMGJointSampler )

DECLARE_NAMESPACE_INTERFACE( EMGProcessing, EMG_PROCESSING_FUNCTIONS );


#endif // EMG_PROCESSING_H 
