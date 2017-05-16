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


#include <string.h>

//#include "debug/sync_debug.h"

#include "data_io.h"

DECLARE_MODULE_INTERFACE( DATA_IO_INTERFACE );
static DataIOImplementation dataIOImplementation = { DEFINE_MODULE_INTERFACE_REF( DATA_IO_INTERFACE ) };

DEFINE_NAMESPACE_INTERFACE( DataIO, DATA_IO_WRAPPER_INTERFACE );


bool DataIO_SetDataType( const char* typeName )
{
  char modulePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  sprintf( modulePath, NAME_STRING( DATA_IO_MODULES_PATH ) "/%s", typeName );
  
  bool moduleLoaded = false;
  LOAD_MODULE_IMPLEMENTATION( DATA_IO_INTERFACE, modulePath, &dataIOImplementation, &moduleLoaded );
  
  return moduleLoaded;
}

DataIOHandler DataIO_GetDataHandler( void )
{
  return &dataIOImplementation;
}

DataHandle CreateEmptyData( void ) { return NULL; }
DataHandle LoadStringData( const char* configString ) { return NULL; }
DataHandle LoadFileData( const char* filePath ) { return NULL; }
void SetBaseFilePath( const char* basePath ) { return (void) 0; }
void UnloadData( DataHandle data ) { return (void) 0; }
char* GetDataString( DataHandle data ) { return NULL; }
DataHandle GetSubData( DataHandle data, const char* pathFormat, ... ) { return NULL; }
char* GetStringValue( DataHandle data, char* defaultValue, const char* pathFormat, ... ) { return defaultValue; }
double GetNumericValue( DataHandle data, double defaultValue, const char* pathFormat, ... ) { return defaultValue; }
bool GetBooleanValue( DataHandle data, bool defaultValue, const char* pathFormat, ... ) { return defaultValue; }
size_t GetListSize( DataHandle data, const char* pathFormat, ... ) { return 0; }
DataHandle AddList( DataHandle data, const char* key ) { return NULL; }
DataHandle AddLevel( DataHandle data, const char* key ) { return NULL; }
bool SetNumericValue( DataHandle data, const char* key, const double value ) { return false; }
bool SetStringValue( DataHandle data, const char* key, const char* value ) { return false; }
bool SetBooleanValue( DataHandle data, const char* key, const bool value ) { return false; }
bool HasKey( DataHandle data, const char* pathFormat, ... ) { return false; }
