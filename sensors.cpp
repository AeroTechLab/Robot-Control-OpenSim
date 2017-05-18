//////////////////////////////////////////////////////////////////////////////////////
//                                                                                  //
//  Copyright (c) 2016-2017 Leonardo Consoni <consoni_2519@hotmail.com>             //
//                                                                                  //
//  This file is part of Robot Control Library.                                     //
//                                                                                  //
//  Robot Control Library is free software: you can redistribute it and/or modify   //
//  it under the terms of the GNU Lesser General Public License as published        //
//  by the Free Software Foundation, either version 3 of the License, or            //
//  (at your option) any later version.                                             //
//                                                                                  //
//  Robot Control Library is distributed in the hope that it will be useful,        //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of                  //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                    //
//  GNU Lesser General Public License for more details.                             //
//                                                                                  //
//  You should have received a copy of the GNU Lesser General Public License        //
//  along with Robot Control Library. If not, see <http://www.gnu.org/licenses/>.   //
//                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////


#include <math.h>
#include <string.h>

#include "signal_io_interface.h"
#include "curve_interpolation.h"

#include "data_logging.h"

#include "sensors.h"


struct _SensorData
{
  DECLARE_MODULE_INTERFACE_REF( SIGNAL_IO_INTERFACE );
  int taskID;
  unsigned int channel;
  double* inputBuffer;
  size_t maxInputSamplesNumber;
  SignalProcessor processor;
  Curve measurementCurve;
  Sensor reference;
  Log log;
};

DEFINE_NAMESPACE_INTERFACE( Sensors, SENSOR_INTERFACE );


Sensor Sensors_Init( DataHandle configuration )
{
  static char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  
  if( configuration == NULL ) return NULL;
  
  char* sensorName = DataIO.GetDataHandler()->GetStringValue( configuration, NULL, "" );
  if( sensorName != NULL )
  {
    sprintf( filePath, "sensors/%s", sensorName );
    if( (configuration = DataIO.GetDataHandler()->LoadFileData( filePath )) == NULL ) return NULL;
  }
  
  //DEBUG_PRINT( "sensor configuration found on data handle %p", configuration );
  
  Sensor newSensor = (Sensor) malloc( sizeof(SensorData) );
  memset( newSensor, 0, sizeof(SensorData) );  
  
  bool loadSuccess;
  sprintf( filePath, NAME_STRING( SIGNAL_IO_MODULES_PATH ) "/%s", DataIO.GetDataHandler()->GetStringValue( configuration, "", "input_interface.type" ) );
  LOAD_MODULE_IMPLEMENTATION( SIGNAL_IO_INTERFACE, filePath, newSensor, &loadSuccess );
  if( loadSuccess )
  {
    newSensor->taskID = newSensor->InitTask( DataIO.GetDataHandler()->GetStringValue( configuration, "", "input_interface.config" ) );
    if( newSensor->taskID != SIGNAL_IO_TASK_INVALID_ID )
    {
      newSensor->channel = (unsigned int) DataIO.GetDataHandler()->GetNumericValue( configuration, -1, "input_interface.channel" );
      loadSuccess = newSensor->AcquireInputChannel( newSensor->taskID, newSensor->channel );
      
      newSensor->maxInputSamplesNumber = newSensor->GetMaxInputSamplesNumber( newSensor->taskID );
      newSensor->inputBuffer = (double*) calloc( newSensor->maxInputSamplesNumber, sizeof(double) );
      
      uint8_t signalProcessingFlags = 0;
      if( DataIO.GetDataHandler()->GetBooleanValue( configuration, false, "signal_processing.rectified" ) ) signalProcessingFlags |= SIGNAL_PROCESSING_RECTIFY;
      if( DataIO.GetDataHandler()->GetBooleanValue( configuration, false, "signal_processing.normalized" ) ) signalProcessingFlags |= SIGNAL_PROCESSING_NORMALIZE;
      newSensor->processor = SignalProcessing.CreateProcessor( signalProcessingFlags );
      
      double inputGain = DataIO.GetDataHandler()->GetNumericValue( configuration, 1.0, "input_gain.multiplier" );
      inputGain /= DataIO.GetDataHandler()->GetNumericValue( configuration, 1.0, "input_gain.divisor" );
      SignalProcessing.SetInputGain( newSensor->processor, inputGain );
      
      double relativeCutFrequency = DataIO.GetDataHandler()->GetNumericValue( configuration, 0.0, "signal_processing.low_pass_frequency" );
      SignalProcessing.SetMaxFrequency( newSensor->processor, relativeCutFrequency );
      
      DataHandle curveConfiguration = DataIO.GetDataHandler()->GetSubData( configuration, "conversion_curve" );
      newSensor->measurementCurve = CurveInterpolation.LoadCurve( curveConfiguration );
      
      char* logFileName = DataIO.GetDataHandler()->GetStringValue( configuration, NULL, "log" );
      if( logFileName != NULL )
      {
        sprintf( filePath, "sensors/%s", logFileName );
        newSensor->log = DataLogging.InitLog( filePath, 4 );
      }
      
      DataHandle referenceConfiguration = DataIO.GetDataHandler()->GetSubData( configuration, "relative_to" );
      newSensor->reference = Sensors_Init( referenceConfiguration );
      if( referenceConfiguration != NULL ) DataIO.GetDataHandler()->UnloadData( referenceConfiguration );
      
      newSensor->Reset( newSensor->taskID );
    }
    else loadSuccess = false;
  }
  
  if( sensorName != NULL ) DataIO.GetDataHandler()->UnloadData( configuration );
  
  if( !loadSuccess )
  {
    Sensors_End( newSensor );
    return NULL;
  }    
  
  return newSensor;
}

void Sensors_End( Sensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->ReleaseInputChannel( sensor->taskID, sensor->channel );
  sensor->EndTask( sensor->taskID );
  
  SignalProcessing.DiscardProcessor( sensor->processor );
  CurveInterpolation.UnloadCurve( sensor->measurementCurve );
  
  free( sensor->inputBuffer );
  
  DataLogging.EndLog( sensor->log );
  
  Sensors_End( sensor->reference );
  
  free( sensor );
}

double Sensors_Update( Sensor sensor, double* rawBuffer )
{
  if( sensor == NULL ) return 0.0;
  
  size_t aquiredSamplesNumber = sensor->Read( sensor->taskID, sensor->channel, sensor->inputBuffer );
  if( rawBuffer != NULL ) memcpy( rawBuffer, sensor->inputBuffer, sensor->maxInputSamplesNumber * sizeof(double) );
    
  double sensorOutput = SignalProcessing.UpdateSignal( sensor->processor, sensor->inputBuffer, aquiredSamplesNumber );
  
  double referenceOutput = Sensors.Update( sensor->reference, NULL );
  //if( sensor->reference != NULL ) DEBUG_PRINT( "sensor: %g - reference: %g", sensorOutput, referenceOutput );
  sensorOutput -= referenceOutput;
  
  double sensorMeasure = CurveInterpolation.GetValue( sensor->measurementCurve, sensorOutput, sensorOutput );
  
  DataLogging.RegisterList( sensor->log, sensor->maxInputSamplesNumber, sensor->inputBuffer );
  DataLogging.RegisterValues( sensor->log, 3, sensorOutput, referenceOutput, sensorMeasure );
  
  return sensorMeasure;
}

size_t Sensors_GetInputBufferLength( Sensor sensor )
{
  if( sensor == NULL ) return 0;
  
  return sensor->maxInputSamplesNumber;
}
  
bool Sensors_HasError( Sensor sensor )
{
  if( sensor == NULL ) return false;
  
  return sensor->HasError( sensor->taskID );
}

void Sensors_Reset( Sensor sensor )
{
  if( sensor == NULL ) return;
  
  SignalProcessing.SetProcessorState( sensor->processor, SIGNAL_PROCESSING_PHASE_MEASUREMENT );
  sensor->Reset( sensor->taskID );
}

void Sensors_SetState( Sensor sensor, enum SignalProcessingPhase newProcessingPhase )
{
  if( sensor == NULL ) return;
  
  SignalProcessing.SetProcessorState( sensor->processor, newProcessingPhase );
  Sensors.SetState( sensor->reference, newProcessingPhase );
}
