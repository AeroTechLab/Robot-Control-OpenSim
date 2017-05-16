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


#ifndef DATA_LOGGING_H
#define DATA_LOGGING_H

#include "namespaces.h"

#define LOG_FILE_PATH_MAX_LEN 256

#define DATA_LOG_MAX_PRECISION 15

typedef struct _LogData LogData;
typedef LogData* Log;

#define DATA_LOGGING_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( Log, Namespace, InitLog, const char*, size_t ) \
        INIT_FUNCTION( void, Namespace, EndLog, Log ) \
        INIT_FUNCTION( void, Namespace, SetBaseDirectory, const char* ) \
        INIT_FUNCTION( void, Namespace, RegisterValues, Log, size_t, ... ) \
        INIT_FUNCTION( void, Namespace, RegisterList, Log, size_t, double* ) \
        INIT_FUNCTION( void, Namespace, RegisterString, Log, const char*, ... ) \
        INIT_FUNCTION( void, Namespace, EnterNewLine, Log )

DECLARE_NAMESPACE_INTERFACE( DataLogging, DATA_LOGGING_INTERFACE );


#endif /* DATA_LOGGING_H */
