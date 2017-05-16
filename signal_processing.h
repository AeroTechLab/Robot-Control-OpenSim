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


/// @file signal_processing.h
/// @brief Common signal processing functions
///
/// Interface for applying common operations to a measured signal, such as amplification, offset measuring/removal, rectification, normalization, filtering, etc.

#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <stdint.h>

#include "namespaces.h"


/// Selectable signal processing phases/modes
enum SignalProcessingPhase 
{ 
  SIGNAL_PROCESSING_PHASE_MEASUREMENT,    ///< Default mode: signal is processed and result is outputed assuming normal operation (preprocessed offset, gain, filter, and limits values are applied)
  SIGNAL_PROCESSING_PHASE_CALIBRATION,    ///< Minimum and maximum values from filtered signal are registered for posterior normalization, if specified
  SIGNAL_PROCESSING_PHASE_OFFSET,         ///< Raw values mean is stored for posterior offset removal (no processed result is outputed)
  SIGNAL_PROCESSING_PHASES_NUMBER         ///< Number of signal processing phases/modes
};

#define SIGNAL_PROCESSING_RECTIFY 0x0F      ///< Create a signal processor that rectifies input signal
#define SIGNAL_PROCESSING_NORMALIZE 0xF0    ///< Create a signal processor that normalizes input signal after calibration

/// Single signal processor (holds single signal processing data) internal data structure
typedef struct _SignalProcessorData SignalProcessorData;
/// Opaque reference to a signal processor data structure
typedef SignalProcessorData* SignalProcessor;

/// Signal processing interface generation macro
#define SIGNAL_PROCESSING_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( SignalProcessor, Namespace, CreateProcessor, uint8_t ) \
        INIT_FUNCTION( void, Namespace, DiscardProcessor, SignalProcessor ) \
        INIT_FUNCTION( void, Namespace, SetInputGain, SignalProcessor, double ) \
        INIT_FUNCTION( void, Namespace, SetMaxFrequency, SignalProcessor, double ) \
        INIT_FUNCTION( double, Namespace, UpdateSignal, SignalProcessor, double*, size_t ) \
        INIT_FUNCTION( double, Namespace, RevertTransformation, SignalProcessor, double ) \
        INIT_FUNCTION( void, Namespace, SetProcessorState, SignalProcessor, enum SignalProcessingPhase ) 

DECLARE_NAMESPACE_INTERFACE( SignalProcessing, SIGNAL_PROCESSING_INTERFACE );


#endif // SIGNAL_PROCESSING_H



/// @struct SignalProcessing
/// @brief Function pointer struct as signal processing interface  
///
/// @memberof SignalProcessing
/// @fn SignalProcessor CreateProcessor( uint8_t flags )                                                             
/// @brief Creates processor data structure based on given flags                                              
/// @param[in] flags set of signal processing options (SIGNAL_PROCESSING_RECTIFY and SIGNAL_PROCESSING_NORMALIZE), that could be omitted (0) or combined (with | operator) 
/// @return reference/pointer to newly created processor data structure
///
/// @memberof SignalProcessing
/// @fn void DiscardProcessor( SignalProcessor processor )
/// @brief Deallocates internal data of given processor                    
/// @param[in] processor reference to signal processor
///
/// @memberof SignalProcessing
/// @fn void SetInputGain( SignalProcessor processor, double inputGain )
/// @brief Sets value that multiplies the input signal before processing
/// @param[in] processor reference to signal processor
/// @param[out] inputGain input gain factor
///
/// @memberof SignalProcessing
/// @fn void SetMaxFrequency( SignalProcessor processor, double relativeFrequency )                                                                    
/// @brief Creates internal low-pass filter for removing processor input signal higher frequencies
/// @param[in] processor reference to signal processor
/// @param[in] relativeFrequency cut frequency, relative to (factor of) input sampling frequency
///
/// @memberof SignalProcessing
/// @fn double UpdateSignal( SignalProcessor processor, double* newInputValuesList, size_t newValuesNumber )
/// @brief Updates processor internal signal value based on given new raw input samples             
/// @param[in] processor reference to signal processor
/// @param[in] newInputValuesList pointer to array of new sampled raw values
/// @param[in] newValuesNumber length (in elements) of the input samples array
/// @return processed (aplified, rectified, filtered, normalized, etc.) resulting signal value
///
/// @memberof SignalProcessing
/// @fn void SetProcessorState( SignalProcessor processor, enum SignalProcessingPhase newProcessingPhase )
/// @brief Sets current processing phase/state/mode of given processor                     
/// @param[in] processor reference to signal processor
/// @param[in] newProcessingPhase desired signal processing phase
///
/// @memberof SignalProcessing
