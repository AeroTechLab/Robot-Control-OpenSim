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


/// @file curve_interpolation.h
/// @brief Abstraction interface for storing curve data
///
/// Interface for calculation of polynomial or spline interpolation between curve points, specified according to @ref curve_config.

/// @page curve_config Curve data configuration
/// The curve configuration is is read using the [data I/O implementation currently defined](https://bitiquinho.github.io/Platform-Utils/structDataIO.html)
///
/// Any configuration file/location path must be provided without its format extension, and relative to CONFIG_DIR/curves/, where CONFIG_DIR is the [defined base data path](https://bitiquinho.github.io/Platform-Utils/classDATA__IO__INTERFACE.html)
///
/// The possible configuration fields and their values are here exemplified for the case of a JSON format configuration:
/// @code
/// {
///   "segments": [                                             // List of segments that compose the entire curve
///     { 
///       "type": "polynomial",                                   // Segment defined by a polynomial expression
///       "bounds": [ -0.5, -0.0032 ],                            // Limits of segment function domain
///       "parameters": [ -652.05, -0.3701 ]                      // Polynom coefficients (from bigger to lower order)
///     },
///     { 
///       "type": "cubic_spline",                                 // Segment defined by cubic (4 coefficients) spline points
///       "bounds": [ -0.0032, 0.0032 ],                          // Limits of segment function domain
///       "parameters": [ 1.7165, -652.05, -1.5608, -671.77 ]     // Function value and derivative (respectively) on each spline bound
///     }
/// }
/// @endcode


#ifndef CURVE_INTERPOLATION_H
#define CURVE_INTERPOLATION_H

#include "data_io.h"

#include "namespaces.h"


typedef struct _CurveData CurveData;      ///< Single curve internal data structure    
typedef CurveData* Curve;                 ///< Opaque reference to curve internal data structure

/// Curve interpolation interface generation macro
#define CURVE_INTERPOLATION_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( Curve, Namespace, LoadCurve, DataHandle ) \
        INIT_FUNCTION( void, Namespace, UnloadCurve, Curve ) \
        INIT_FUNCTION( void, Namespace, SetScale, Curve, double ) \
        INIT_FUNCTION( void, Namespace, SetOffset, Curve, double ) \
        INIT_FUNCTION( double, Namespace, GetValue, Curve, double, double )

DECLARE_NAMESPACE_INTERFACE( CurveInterpolation, CURVE_INTERPOLATION_INTERFACE );


#endif  // CURVE_INTERPOLATION_H



/// @struct CurveInterpolation
/// @brief Function pointer struct as curve interpolation interface  
///
/// @memberof CurveInterpolation
/// @fn Curve LoadCurve( DataHandle curveData )                                                                        
/// @brief Creates curve data structure and loads segments data                                              
/// @param[in] curveData reference to curve configuration data structure, as explained at @ref curve_config
/// @return reference/pointer to newly created and filled curve data structure
///
/// @memberof CurveInterpolation
/// @fn void UnloadCurve( Curve curve )
/// @brief Deallocates internal data of given curve                        
/// @param[in] curve reference to curve
///
/// @memberof CurveInterpolation
/// @fn double GetValue( Curve curve, double valuePosition, double defaultValue )
/// @brief Gets given curve value on specified position
/// @param[in] curve reference to curve
/// @param[in] valuePosition position (x coordinate) where curve value will be calculated
/// @param[in] defaultValue value to be returned if curve value can't be obtained
/// @return calculated curve value or default one
///
/// @memberof CurveInterpolation
/// @fn void SetScale( Curve curve, double scaleFactor )
/// @brief Sets multiplier to be applied to calculated curve value before offset (y axis scaling)
/// @param[in] curve reference to curve
/// @param[out] scaleFactor factor that will multiply next curve values
///
/// @memberof CurveInterpolation
/// @fn void SetOffset( Curve curve, double offset )                                                                  
/// @brief Sets given curve offset (y axis displacement)
/// @param[in] curve reference to curve
/// @param[in] offset value to add to calculated curve value
///
/// @memberof CurveInterpolation
/// @fn void SetMaxAmplitude( Curve curve, double maxAmplitude )
/// @brief Limit maximum absolute value calculated for given curve        
/// @param[in] curve reference to curve
/// @param[in] maxAmplitude maximum absolute value ( less than zero for no limit)
///
/// @memberof CurveInterpolation
