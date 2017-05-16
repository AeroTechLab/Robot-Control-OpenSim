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


/// @file data_io_interface.h
/// @brief Data read/write functions.
///
/// Common data file/string parsing/saving interface to be implemented by plugins.

#ifndef DATA_IO_H
#define DATA_IO_H

#include "modules.h"

#include <stdarg.h>

#define DATA_IO_MAX_FILE_PATH_LENGTH 256    ///< Maximum length of data file path (from base directory) string
#define DATA_IO_MAX_KEY_PATH_LENGTH 256     ///< Maximum length of value key/index string inside data structure
#define DATA_IO_MAX_VALUE_LENGTH 128        ///< Maximum length of value string inside data structure

typedef void* DataHandle;                   ///< Opaque reference to internal data structure

/// Data read/write interface declaration macro
#define DATA_IO_INTERFACE( Interface, INIT_FUNCTION ) \
        INIT_FUNCTION( DataHandle, Interface, CreateEmptyData, void ) \
        INIT_FUNCTION( DataHandle, Interface, LoadFileData, const char* ) \
        INIT_FUNCTION( void, Interface, SetBaseFilePath, const char* ) \
        INIT_FUNCTION( DataHandle, Interface, LoadStringData, const char* ) \
        INIT_FUNCTION( void, Interface, UnloadData, DataHandle ) \
        INIT_FUNCTION( char*, Interface, GetDataString, DataHandle ) \
        INIT_FUNCTION( DataHandle, Interface, GetSubData, DataHandle, const char*, ... ) \
        INIT_FUNCTION( double, Interface, GetNumericValue, DataHandle, double, const char*, ... ) \
        INIT_FUNCTION( char*, Interface, GetStringValue, DataHandle, char*, const char*, ... ) \
        INIT_FUNCTION( bool, Interface, GetBooleanValue, DataHandle, bool, const char*, ... ) \
        INIT_FUNCTION( size_t, Interface, GetListSize, DataHandle, const char*, ... ) \
        INIT_FUNCTION( DataHandle, Interface, AddList, DataHandle, const char* ) \
        INIT_FUNCTION( DataHandle, Interface, AddLevel, DataHandle, const char* ) \
        INIT_FUNCTION( bool, Interface, SetNumericValue, DataHandle, const char*, const double ) \
        INIT_FUNCTION( bool, Interface, SetStringValue, DataHandle, const char*, const char* ) \
        INIT_FUNCTION( bool, Interface, SetBooleanValue, DataHandle, const char*, const bool ) \
        INIT_FUNCTION( bool, Interface, HasKey, DataHandle, const char*, ... )

#endif // DATA_IO_H
        
        
        
/// @class DATA_IO_INTERFACE
/// @brief File/string/stream data input/output methods to be implemented by plugins
///        
/// @memberof DATA_IO_INTERFACE
/// @fn DataHandle CreateEmptyData( void )                                                                             
/// @brief Creates plugin specific empty data structure object 
/// @return reference/pointer to newly created internal data structure (NULL on errors)
///        
/// @memberof DATA_IO_INTERFACE
/// @fn DataHandle LoadFileData( const char* filePath )
/// @brief Loads all given storage to fill plugin specific data structure
/// @param[in] filePath path (from base directory) to data file
/// @return reference/pointer to created and filled data structure (NULL on errors)
///
/// @memberof DATA_IO_INTERFACE
/// @fn void SetBaseFilePath( const char* basePath )
/// @brief Overwrites default root path from which configuration files will be searched                              
/// @param[in] basePath path (absolute or relative) to desired root data file path
///        
/// @memberof DATA_IO_INTERFACE
/// @fn int LoadStringData( const char* dataString )                                                                                 
/// @brief Parses given string to fill plugin specific data structure
/// @param[in] dataString string containing data to be parsed
/// @return reference/pointer to created and filled data structure (NULL on errors)
///        
/// @memberof DATA_IO_INTERFACE        
/// @fn void UnloadData( DataHandle data )
/// @brief Deallocates and destroys given data structure
/// @param[in] data reference to internal data structure
///        
/// @memberof DATA_IO_INTERFACE
/// @fn char* GetDataString( DataHandle data )
/// @brief Get given data structure content in serialized string form
/// @param[in] data reference to internal data structure to be serialized
/// @return allocated pointer to serialized data string (needs to be manually deallocated)
///        
/// @memberof DATA_IO_INTERFACE
/// @fn DataHandle GetSubData( DataHandle data, const char* pathFormat, ... )
/// @brief Get reference to inner data level from given data strucuture
/// @param[in] data reference to internal data structure where the value will be searched
/// @param[in] pathFormat format string (like in printf) to value path inside the data structure (key or index fields separated by ".")
/// @param[in] ... list of string keys or numeric indexes to build searched value path from pathFormat (like in printf)
/// @return reference/pointer to internal data structure (NULL on errors)
///        
/// @memberof DATA_IO_INTERFACE
/// @fn double GetNumericValue( DataHandle data, double defaultValue, const char* pathFormat, ... )
/// @brief Get specified numeric value (floating point format) from given data strucuture
/// @param[in] data reference to internal data structure where the value will be searched
/// @param[in] defaultValue value to be returned if specified field is not found
/// @param[in] pathFormat format string (like in printf) to value path inside the data structure (key or index fields separated by ".")
/// @param[in] ... list of string keys or numeric indexes to build searched value path from pathFormat (like in printf)
/// @return numeric value (floating point format) found or the default one
///        
/// @memberof DATA_IO_INTERFACE
/// @fn char* GetStringValue( DataHandle data, char* defaultValue, const char* pathFormat, ... )
/// @brief Get specified string value from given data strucuture
/// @param[in] data reference to internal data structure where the value will be searched
/// @param[in] defaultValue value to be returned if specified field is not found
/// @param[in] pathFormat format string (like in printf) to value path inside the data structure (key or index fields separated by ".")
/// @param[in] ... list of string keys or numeric indexes to build searched value path from pathFormat (like in printf)
/// @return string value found or the default one
///        
/// @memberof DATA_IO_INTERFACE
/// @fn bool GetBooleanValue( DataHandle data, bool defaultValue, const char* pathFormat, ... )
/// @brief Get specified boolean value from given data strucuture
/// @param[in] data reference to internal data structure where the value will be searched
/// @param[in] defaultValue value to be returned if specified field is not found
/// @param[in] pathFormat format string (like in printf) to value path inside the data structure (key or index fields separated by ".")
/// @param[in] ... list of string keys or numeric indexes to build searched value path from pathFormat (like in printf)
/// @return boolean value found or the default one
///        
/// @memberof DATA_IO_INTERFACE
/// @fn size_t GetListSize( DataHandle data, const char* pathFormat, ... )
/// @brief Get number of elements for specified list from given data strucuture
/// @param[in] data reference to internal data structure where the list will be searched
/// @param[in] pathFormat format string (like in printf) to list path inside the data structure (key or index fields separated by ".")
/// @param[in] ... list of string keys or numeric indexes to build searched list path from pathFormat (like in printf)
/// @return number of elements of the list (or 0 if list is not found)
///        
/// @memberof DATA_IO_INTERFACE
/// @fn DataHandle AddList( DataHandle data, const char* key )
/// @brief Insert list on specified field of given data strucuture
/// @param[in] data reference to internal data structure where the list will be placed
/// @param[in] key string identifier of the field where the list will be placed (NULL for appending to list)
/// @return reference/pointer to newly created internal data structure (NULL on errors)
///        
/// @memberof DATA_IO_INTERFACE        
/// @fn DataHandle AddLevel( DataHandle data, const char* key )
/// @brief Insert nesting level on specified field of given data strucuture
/// @param[in] data reference to internal data structure where the nesting level will be added
/// @param[in] key string identifier of the field where the nesting level will be added (NULL for appending to list)
/// @return reference/pointer to newly created internal data structure (NULL on errors)
///        
/// @memberof DATA_IO_INTERFACE
/// @fn bool SetNumericValue( DataHandle data, const char* key, const double value )
/// @brief Set numeric value (floating point format) for specified field of given data strucuture
/// @param[in] data reference to internal data structure where the value will be placed/updated
/// @param[in] key string identifier of the field where the value will be placed/updated (NULL for appending to list)
/// @param[in] value numeric value to be inserted/updated on given data structure field
/// @return true if value is inserted successfully, false otherwise
///        
/// @memberof DATA_IO_INTERFACE
/// @fn bool SetStringValue( DataHandle data, const char* key, const char* value )
/// @brief Set string value (floating point format) for specified field of given data strucuture
/// @param[in] data reference to internal data structure where the value will be placed/updated
/// @param[in] key string identifier of the field where the value will be placed/updated (NULL for appending to list)
/// @param[in] value string value to be inserted/updated on given data structure field
/// @return true if value is inserted successfully, false otherwise
///        
/// @memberof DATA_IO_INTERFACE        
/// @fn bool SetBooleanValue( DataHandle data, const char* key, const bool value )
/// @brief Set boolean value (floating point format) for specified field of given data strucuture
/// @param[in] data reference to internal data structure where the value will be placed/updated
/// @param[in] key string identifier of the field where the value will be placed/updated (NULL for appending to list)
/// @param[in] value boolean value to be inserted/updated on given data structure field
/// @return true if value is inserted successfully, false otherwise
///        
/// @memberof DATA_IO_INTERFACE       
/// @fn bool HasKey( DataHandle data, const char* pathFormat, ... )
/// @brief Verify if specified value field/key is present inside given data strucuture
/// @param[in] data reference to internal data structure where the key will be searched
/// @param[in] pathFormat format string (like in printf) to key path inside the data structure (key or index fields separated by ".")
/// @param[in] ... list of string keys or numeric indexes to build searched path from pathFormat (like in printf)
/// @return true if key is found, false otherwise
///
/// @memberof DATA_IO_INTERFACE
