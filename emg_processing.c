////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016 Leonardo José Consoni                                  //
//                                                                            //
//  This file is part of RobRehabSystem.                                      //
//                                                                            //
//  RobRehabSystem is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobRehabSystem is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#include <math.h>

#include "sensors.h"
#include "signal_processing.h"
//#include "curve_interpolation.h"

#include "data_io.h"

#include "data_logging.h"

#include "emg_processing.h"

enum { MUSCLE_ACTIVE_FORCE, MUSCLE_PASSIVE_FORCE, MUSCLE_MOMENT_ARM, MUSCLE_NORM_LENGTH, MUSCLE_CURVES_NUMBER };

typedef struct _EMGMuscleData
{
  Sensor emgSensor;
  double* emgRawBuffer;
  size_t emgRawBufferLength;
  //Curve curvesList[ MUSCLE_CURVES_NUMBER ];
  double activationFactor;
  double scalingFactor;
  double initialPenationAngle;
}
EMGMuscleData;

typedef EMGMuscleData* EMGMuscle;

typedef struct _EMGJointData
{
  EMGMuscle* musclesList;
  size_t musclesListLength;
  //Log offsetLog, calibrationLog, samplingLog;
  //Log currentLog;
  //Log emgRawLog;
}
EMGJointData;


DEFINE_NAMESPACE_INTERFACE( EMGProcessing, EMG_PROCESSING_FUNCTIONS );


const char* MUSCLE_CURVE_NAMES[ MUSCLE_CURVES_NUMBER ] = { "active_force", "passive_force", "moment_arm", "normalized_length" };
static EMGMuscle LoadEMGMuscleData( DataHandle modelData )
{
  if( modelData != NULL )
  {
    EMGMuscle newMuscle = (EMGMuscle) malloc( sizeof(EMGMuscleData) );
    
    //for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
    //{
      //DataHandle curveData = DataIO.GetDataHandler()->GetSubData( modelData, "%s", MUSCLE_CURVE_NAMES[ curveIndex ] );
      //newMuscle->curvesList[ curveIndex ] = CurveInterpolation.LoadCurve( curveData );
    //}

    newMuscle->activationFactor = DataIO.GetDataHandler()->GetNumericValue( modelData, -2.0, "activation_factor" );
    newMuscle->scalingFactor = DataIO.GetDataHandler()->GetNumericValue( modelData, 1.0, "scaling_factor" );
    newMuscle->initialPenationAngle = DataIO.GetDataHandler()->GetNumericValue( modelData, 0.0, "initial_penation_angle" );
    
    return newMuscle;
  }
  
  return NULL;
}

EMGJoint EMGProcessing_InitJoint( const char* configFileName )
{
  static char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  
  //DEBUG_PRINT( "Trying to load joint %s EMG data", configFileName );
  
  EMGJoint newJoint = NULL;
  
  sprintf( filePath, "joints/%s", configFileName );
  DataHandle configuration = DataIO.GetDataHandler()->LoadFileData( filePath );
  if( configuration != NULL )
  {
    newJoint = (EMGJoint) malloc( sizeof(EMGJointData) );
    memset( newJoint, 0, sizeof(EMGJointData) );
    
    bool loadError = false;
    size_t emgRawSamplesNumber = 0;
    if( (newJoint->musclesListLength = (size_t) DataIO.GetDataHandler()->GetListSize( configuration, "muscles" )) > 0 )
    {
      //DEBUG_PRINT( "%u muscles found for joint %s", newJoint->musclesListLength, configFileName );
      
      newJoint->musclesList = (EMGMuscle*) calloc( newJoint->musclesListLength , sizeof(EMGMuscle) );
      for( size_t muscleIndex = 0; muscleIndex < newJoint->musclesListLength; muscleIndex++ )
      {
        DataHandle modelData = DataIO.GetDataHandler()->GetSubData( configuration, "muscles.%lu.model_functions", muscleIndex );
        newJoint->musclesList[ muscleIndex ] = LoadEMGMuscleData( modelData );
        if( newJoint->musclesList[ muscleIndex ] != NULL )
        {
          DataHandle sensorData = DataIO.GetDataHandler()->GetSubData( configuration, "muscles.%lu.emg_sensor", muscleIndex );
          newJoint->musclesList[ muscleIndex ]->emgSensor = Sensors.Init( sensorData );
          if( newJoint->musclesList[ muscleIndex ]->emgSensor != NULL )
          {
            newJoint->musclesList[ muscleIndex ]->emgRawBufferLength = Sensors.GetInputBufferLength( newJoint->musclesList[ muscleIndex ]->emgSensor );
            newJoint->musclesList[ muscleIndex ]->emgRawBuffer = (double*) calloc( newJoint->musclesList[ muscleIndex ]->emgRawBufferLength, sizeof(double) );
            emgRawSamplesNumber += newJoint->musclesList[ muscleIndex ]->emgRawBufferLength;
          }
          else
            loadError = true;
        }
        else loadError = true;
      }
      
      /*char* logName = DataIO.GetDataHandler()->GetStringValue( configuration, NULL, "log" );
      if( logName != NULL )
      {
        size_t jointSampleValuesNumber = newJoint->musclesListLength + 3;
        
        snprintf( filePath, LOG_FILE_PATH_MAX_LEN, "joints/%s_offset", logName );                                    
        newJoint->offsetLog = DataLogging.InitLog( filePath, jointSampleValuesNumber, DATA_LOG_MAX_PRECISION );
        snprintf( filePath, LOG_FILE_PATH_MAX_LEN, "joints/%s_calibration", logName );
        newJoint->calibrationLog = DataLogging.InitLog( filePath, jointSampleValuesNumber, DATA_LOG_MAX_PRECISION );
        snprintf( filePath, LOG_FILE_PATH_MAX_LEN, "joints/%s_sampling", logName );
        newJoint->samplingLog = DataLogging.InitLog( filePath, jointSampleValuesNumber, DATA_LOG_MAX_PRECISION );
        snprintf( filePath, LOG_FILE_PATH_MAX_LEN, "joints/%s_raw", logName );
        newJoint->emgRawLog = DataLogging.InitLog( filePath, emgRawSamplesNumber + 1, 6 );
      }*/
    }
    else loadError = true;
    
    DataIO.GetDataHandler()->UnloadData( configuration );
    
    if( loadError )
    {
      EMGProcessing_EndJoint( newJoint );
      return NULL;
    }
  }
  //else
  //  DEBUG_PRINT( "configuration for joint %s not found", configFileName );
  
  return newJoint;
}

void EMGProcessing_EndJoint( EMGJoint joint )
{
  if( joint == NULL ) return;
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
  {
    Sensors.End( joint->musclesList[ muscleIndex ]->emgSensor );
    //for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
    //  CurveInterpolation.UnloadCurve( joint->musclesList[ muscleIndex ]->curvesList[ curveIndex ] );
    free( joint->musclesList[ muscleIndex ]->emgRawBuffer );
  }
  
  //DataLogging.EndLog( joint->offsetLog );
  //DataLogging.EndLog( joint->calibrationLog );
  //DataLogging.EndLog( joint->samplingLog );
  //DataLogging.EndLog( joint->emgRawLog );
  
  free( joint->musclesList );
  
  free( joint );
}

void EMGProcessing_GetJointMuscleSignals( EMGJoint joint, double* normalizedSignalsList )
{
  if( joint == NULL ) return;
  
  //DEBUG_PRINT( "updating sensor %d-%lu (%p)", jointID, muscleIndex, joint->musclesList[ muscleIndex ]->emgSensor );
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
    normalizedSignalsList[ muscleIndex ] = Sensors.Update( joint->musclesList[ muscleIndex ]->emgSensor, joint->musclesList[ muscleIndex ]->emgRawBuffer );
  
  /*if( joint->currentLog != NULL && joint->emgRawLog != NULL )
  {
    DataLogging.RegisterValues( joint->emgRawLog, 1, Timing.GetExecTimeSeconds() );
    for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
      DataLogging.RegisterList( joint->emgRawLog, joint->musclesList[ muscleIndex ]->emgRawBufferLength, joint->musclesList[ muscleIndex ]->emgRawBuffer );
  }*/
}

void EMGProcessing_GetJointMuscleTorques( EMGJoint joint, double* normalizedSignalsList, double jointAngle, double jointExternalTorque, double* emgSamplesList )
{
  if( joint == NULL ) return;
  
  /*if( joint->currentLog != NULL )
  {
    DataLogging.RegisterValues( joint->currentLog, 3, Timing.GetExecTimeSeconds(), jointAngle, jointExternalTorque );
    DataLogging.RegisterList( joint->currentLog, joint->musclesListLength, normalizedSignalsList );
  }*/
  
  //DEBUG_PRINT( "Muscle signals: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", normalizedSignalsList[ 0 ], normalizedSignalsList[ 1 ], normalizedSignalsList[ 2 ], normalizedSignalsList[ 3 ], normalizedSignalsList[ 4 ], normalizedSignalsList[ 5 ] );
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
  {
    EMGMuscle muscle = joint->musclesList[ muscleIndex ];
    
    double activation = ( exp( muscle->activationFactor * normalizedSignalsList[ muscleIndex ] ) - 1 ) / ( exp( muscle->activationFactor ) - 1 );
  
    //double activeForce = CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_ACTIVE_FORCE ], jointAngle, 0.0 );
    //double passiveForce = CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_PASSIVE_FORCE ], jointAngle, 0.0 );
  
    //double normalizedLength = CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_NORM_LENGTH ], jointAngle, 0.0 );
    //double momentArm = CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_MOMENT_ARM ], jointAngle, 0.0 );
  
    //if( normalizedLength == 0.0 ) normalizedLength = 1.0;
    //double penationAngle = asin( sin( muscle->initialPenationAngle ) / normalizedLength );
  
    //double normalizedForce = activeForce * activation + passiveForce;
    //double resultingForce = muscle->scalingFactor * cos( penationAngle ) * normalizedForce;
  
    //normalizedSignalsList[ muscleIndex ] = resultingForce * momentArm;
  }
}

double EMGProcessing_GetJointTorque( EMGJoint joint, double* muscleTorquesList )
{
  if( joint == NULL ) return 0.0;
  
  double jointTorque = 0.0;

  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
    jointTorque += muscleTorquesList[ muscleIndex ];
  
  return jointTorque;
}

double EMGProcessing_GetJointStiffness( EMGJoint joint, double* muscleTorquesList )
{
  if( joint == NULL ) return 0.0;
  
  double jointStiffness = 0.0;
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
    jointStiffness += fabs( muscleTorquesList[ muscleIndex ] );
  
  //DEBUG_PRINT( "Muscle torque: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", muscleTorquesList[ 0 ], muscleTorquesList[ 1 ], muscleTorquesList[ 2 ], muscleTorquesList[ 3 ], muscleTorquesList[ 4 ], muscleTorquesList[ 5 ] );
  
  return jointStiffness;
}

void EMGProcessing_SetJointProcessingPhase( EMGJoint joint, enum EMGProcessingPhase processingPhase )
{
  if( joint == NULL ) return;
  
  enum SignalProcessingPhase signalProcessingPhase = SIGNAL_PROCESSING_PHASE_MEASUREMENT;
  
  if( processingPhase == EMG_PROCESSING_OFFSET )
  {
    signalProcessingPhase = SIGNAL_PROCESSING_PHASE_OFFSET;
    //joint->currentLog = joint->offsetLog;
  }
  else if( processingPhase == EMG_PROCESSING_CALIBRATION )
  {
    signalProcessingPhase = SIGNAL_PROCESSING_PHASE_CALIBRATION;
    //joint->currentLog = joint->calibrationLog;
  }
  //else if( processingPhase == EMG_PROCESSING_SAMPLING )
  //  joint->currentLog = joint->samplingLog;
  //else // if( processingPhase == EMG_PROCESSING_MEASUREMENT )
  //  joint->currentLog = NULL;
  
  //DEBUG_PRINT( "new EMG processing phase: %d (log: %p)", processingPhase, joint->currentLog );
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
    Sensors.SetState( joint->musclesList[ muscleIndex ]->emgSensor, signalProcessingPhase );
}

size_t EMGProcessing_GetJointMusclesCount( EMGJoint joint )
{
  if( joint == NULL ) return 0;
  
  return joint->musclesListLength;
}

void EMGProcessing_FitJointParameters( EMGJoint joint, EMGJointSampler samplingData )
{
  /*if( joint == NULL ) return;
  
  double squaredErrorSum = 0;
  size_t musclesNumber = joint->musclesListLength;
  
  double squaredErrorMean = 1000.0;
  while( squaredErrorMean > 0.1 )
  {
    for( size_t muscleIndex = 0; muscleIndex < musclesNumber; muscleIndex++ )
    {
      for( size_t muscleParameterIndex = 0; muscleParameterIndex < MUSCLE_GAINS_NUMBER; muscleParameterIndex++ )
      {
        double currentValue = parametersList[ muscleIndex * MUSCLE_GAINS_NUMBER + muscleParameterIndex ];
        currentValue = EMGProcessing.SetJointMuscleGain( sampler->jointID, muscleIndex, muscleParameterIndex, currentValue );
        parametersList[ muscleIndex * MUSCLE_GAINS_NUMBER + muscleParameterIndex ] = currentValue;
      }
    
      double* muscleSignalList = sampler->muscleSignalsList[ muscleIndex ];
    
      for( size_t sampleIndex = 0; sampleIndex < sampler->samplesCount; sampleIndex++ )
      {
        double normalizedSample = muscleSignalList[ sampleIndex ];
        double jointAngle = sampler->jointAnglesList[ sampleIndex ];
        double jointIDTorque = sampler->jointIDTorquesList[ sampleIndex ];
    
        double jointEMGTorque = EMGProcessing.GetJointMuscleTorque( sampler->jointID, muscleIndex, normalizedSample, jointAngle );
      
        double sampleError = jointEMGTorque - jointIDTorque;
      
        squaredErrorSum += ( sampleError * sampleError );
      }
    }
  
    squaredErrorMean = squaredErrorSum / sampler->samplesCount;
  }*/
}
