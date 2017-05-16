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


/// @file data_io.h
/// @brief Persistent data access wrapper interface.
///
/// Functions for accessing stored configurations at default folders/urls 
/// Uses plugin specifc underlying implementations for file/string/stream/database access       

#ifndef FILE_IO_H
#define FILE_IO_H

#include "namespaces.h"
#include "modules.h"

#include "data_io_interface.h"

//#ifdef WIN32
//  #include <direct.h>
//  #define SET_PATH( dirPath ) _chdir( dirPath );
//#else
//  #include <unistd.h>
//  #define SET_PATH( dirPath ) chdir( dirPath );
//#endif

/// @def SET_PATH( dirpath )
/// @brief Set the current base directory path to @a dirPath

/// Function pointer struct holding underlying plugin DATA_IO_INTERFACE implementation for configuration storage access (read/write)
typedef struct
{
  DECLARE_MODULE_INTERFACE_REF( DATA_IO_INTERFACE );
}
DataIOImplementation;
/// Opaque type for configuration read/write interface implementation structure
typedef DataIOImplementation* DataIOHandler;

/// Data I/O wrapper interface declaration macro   
#define DATA_IO_WRAPPER_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( bool, Namespace, SetDataType, const char* ) \
        INIT_FUNCTION( DataIOHandler, Namespace, GetDataHandler, void )        

DECLARE_NAMESPACE_INTERFACE( DataIO, DATA_IO_WRAPPER_INTERFACE );


#endif // FILE_IO_H


/// @struct DataIO
/// @brief Function pointer struct as persistent configuration interface 
///
/// @memberof DataIO
/// @fn bool SetDataType( const char* dataType )                                                                               
/// @brief Loads plugin specific DATA_IO_INTERFACE implementation according to given type 
/// @param[in] dataType data format type (used as implementation plugin/library name)
/// @return true on successful loading, false otherwise  
///        
/// @memberof DataIO
/// @fn DataIOHandler GetDataHandler( void )
/// @brief Gives reference to underlying DATA_IO_INTERFACE implementation for direct access                              
/// @return reference to underlying DATA_IO_INTERFACE implementation (loaded one or initial dummy one) 
///
/// @memberof DataIO
