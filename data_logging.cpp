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


#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>

#include "timing.h"
//#include "debug/async_debug.h"

#include "data_logging.h"

#define TIME_STAMP_STRING_LENGTH 32


struct _LogData
{
  FILE* file;
  int dataPrecision;
};

static char baseDirectoryPath[ LOG_FILE_PATH_MAX_LEN ] = "";
static char timeStampString[ TIME_STAMP_STRING_LENGTH ] = "";


DEFINE_NAMESPACE_INTERFACE( DataLogging, DATA_LOGGING_INTERFACE );


Log DataLogging_InitLog( const char* filePath, size_t dataPrecision )
{
  static char filePathExt[ LOG_FILE_PATH_MAX_LEN ];
  
  sprintf( filePathExt, "logs/%s/%s-%s.log", baseDirectoryPath, filePath, timeStampString );

  Log newLog = (Log) malloc( sizeof(LogData) );
  
  //DEBUG_PRINT( "trying to open file %s", filePathExt );
  if( (newLog->file = fopen( filePathExt, "w+" )) == NULL )
  {
    perror( "error opening file" );
    DataLogging_EndLog( newLog );
    return NULL;
  }
  
  newLog->dataPrecision = ( dataPrecision < DATA_LOG_MAX_PRECISION ) ? dataPrecision : DATA_LOG_MAX_PRECISION;
  
  return newLog;
}

void DataLogging_EndLog( Log log )
{
  if( log == NULL ) return;
  
  if( log->file != NULL ) 
  {
    fflush( log->file );
    fclose( log->file );
  }
  
  free( log );
}

void DataLogging_SetBaseDirectory( const char* directoryPath )
{
  time_t timeStamp = time( NULL );
  strncpy( timeStampString, asctime( localtime( &timeStamp ) ), TIME_STAMP_STRING_LENGTH );
  for( size_t charIndex = 0; charIndex < TIME_STAMP_STRING_LENGTH; charIndex++ )
  {
    char c = timeStampString[ charIndex ];
    if( c == ' ' || c == ':' ) timeStampString[ charIndex ] = '_';
    else if( c == '\n' || c == '\r' ) timeStampString[ charIndex ] = '\0';
  }
  
  strncpy( baseDirectoryPath, ( directoryPath != NULL ) ? directoryPath : "", LOG_FILE_PATH_MAX_LEN );
}

void DataLogging_RegisterValues( Log log, size_t valuesNumber, ... )
{
  if( log == NULL ) return;
  
  va_list logValues;
  
  va_start( logValues, valuesNumber );

  for( size_t valueListIndex = 0; valueListIndex < valuesNumber; valueListIndex++ )
    fprintf( log->file, "%.*lf\t", log->dataPrecision, va_arg( logValues, double ) );

  va_end( logValues );
}

void DataLogging_RegisterList( Log log, size_t valuesNumber, double* valuesList )
{
  if( log == NULL ) return;

  for( size_t valueListIndex = 0; valueListIndex < valuesNumber; valueListIndex++ )
    fprintf( log->file, "%.*lf\t", log->dataPrecision, valuesList[ valueListIndex ] );
}

void DataLogging_RegisterString( Log log, const char* formatString, ... )
{
  if( log == NULL ) return;
  
  va_list logValues;
  
  va_start( logValues, formatString );

  vfprintf( log->file, formatString, logValues );

  va_end( logValues );
}

void DataLogging_EnterNewLine( Log log )
{
	if( log == NULL ) return;

	if( ftell( log->file ) > 0 ) fprintf( log->file, "\n" );

	fprintf( log->file, "%f", Timing.GetExecTimeSeconds() );
}