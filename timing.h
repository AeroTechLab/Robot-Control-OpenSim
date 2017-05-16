////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2017 Leonardo Consoni <consoni_2519@hotmail.com>       //
//                                                                            //
//  This file is part of Platform Utils.                                      //
//                                                                            //
//  Platform Utils is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  Platform Utils is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with Platform Utils. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


/// @file time/timing.h
/// @brief Platform agnostic time management functions.
///
/// Wrapper library for time measurement and thread sleeping (blocking) 
/// abstracting underlying low level operating system native methods                     


#ifndef TIMING_H
#define TIMING_H

#include "namespaces.h"

/// Time management interface generation macro
#define TIMING_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( void, Namespace, Delay, unsigned long ) \
        INIT_FUNCTION( unsigned long, Namespace, GetExecTimeMilliseconds, void ) \
        INIT_FUNCTION( double, Namespace, GetExecTimeSeconds, void )

DECLARE_NAMESPACE_INTERFACE( Timing, TIMING_INTERFACE );

#endif // TIMING_H


/// @struct Timing
/// @brief Function pointer struct as timing interface  
///
/// @memberof Timing
/// @fn void Delay( unsigned long milliseconds )                                                                             
/// @brief Makes the calling thread wait for the specified time                                           
/// @param[in] milliseconds time interval (in milliseconds) during which the calling thread will block/sleep                                   
///
/// @memberof Timing
/// @fn unsigned long GetExecTimeMilliseconds()
/// @brief Gets system time in milisseconds                             
/// @return system clock ticks count converted to milliseconds 
///
/// @memberof Timing
/// @fn double GetExecTimeSeconds()
/// @brief Gets system time in seconds                              
/// @return system clock ticks count converted to seconds
///
/// @memberof Timing
