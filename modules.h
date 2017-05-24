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


/// @file modules.h
/// @brief Platform abstraction macros for plugins/modules creation and loading.
///
/// Automates generation and loading of exported Functiontion symbols of dynamic libraries,
/// according to an interface definition (for more details on interface declaration, see namespaces.h)

#ifndef MODULES_H
#define MODULES_H

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>

#ifndef __cplusplus
#if __STDC_VERSION__ >= 199901L || __cplusplus >= 201103L
#include <stdbool.h>
#else
#define false   0
#define true    1
#define bool char
#endif
#endif

#ifdef WIN32
  #include <windows.h>

  #define LOAD_PLUGIN( pluginPath ) LoadLibrary( pluginPath )
  #define LOAD_PLUGIN_SYMBOL( pluginHandle, functionName ) GetProcAddress( pluginHandle, functionName )
  #define UNLOAD_PLUGIN( pluginHandle ) FreeLibrary( pluginHandle )
  
  #define PLUGIN_HANDLE HINSTANCE
  #define PLUGIN_EXTENSION "dll"
#elif __unix__
  #include <stdint.h>
  #include <dlfcn.h>
  #include <unistd.h>
  #define __declspec(dllexport)
      
  #define LOAD_PLUGIN( pluginPath ) dlopen( pluginPath, RTLD_NOW )
  #define LOAD_PLUGIN_SYMBOL( pluginHandle, functionName ) ( (intptr_t) dlsym( pluginHandle, functionName ) )
  #define UNLOAD_PLUGIN( pluginHandle ) dlclose( pluginHandle ) 
  
  #define PLUGIN_HANDLE void*
  #define PLUGIN_EXTENSION "so" 
#endif

#ifdef __cplusplus 
  #define C_FUNCTION extern "C"
#else
  #define C_FUNCTION extern
#endif

/// @def LOAD_PLUGIN( pluginPath )
/// @brief Calls platform specific code to load dynamic library from @a pluginPath
///
/// @def LOAD_PLUGIN_SYMBOL( pluginHandle, functionName )
/// @brief Calls platform specific code to load Functiontion @a functionName symbol from loaded library handle @a pluginHandle
///
/// @def UNLOAD_PLUGIN( pluginHandle )
/// @brief Calls platform specific code to unload loaded library handle @a pluginHandle
///
/// @def PLUGIN_HANDLE
/// @brief Platform specific dynamic library handle
///
/// @def PLUGIN_EXTENSION
/// @brief Platform specific dynamic library file extension


#define UNPACK_NAME( name ) # name
#define NAME_STRING( name ) UNPACK_NAME( name )    ///< Converts Functiontion name @a name to string

/// Automates declaration of exported symbol library Functiontion
#define DECLARE_MODULE_FUNCTION( rtype, Module, Function, ... ) C_FUNCTION __declspec(dllexport) rtype Function( __VA_ARGS__ );
/// Automates declaration of Functiontion pointer from
#define DECLARE_MODULE_FUNC_REF( rtype, Module, Function, ... ) rtype (*Function)( __VA_ARGS__ );
/// Automates definition of interface struct Functiontion field
#if __STDC_VERSION__ >= 199901L
#define DEFINE_MODULE_FUNC_REF( rtype, Module, Function, ... ) .Function = Function,
#else
#define DEFINE_MODULE_FUNC_REF( rtype, Module, Function, ... ) Function,
#endif

/// Automates definition of dummy Functiontion implementation
#define DEFINE_MODULE_FUNC_STUB( rtype, Module, Function, ... ) rtype Function( __VA_ARGS__ ) { return (rtype) 0; }

/// Automates loading of a library Functiontion symbol and its attribuition to a correspondent interface struct Functiontion field 
#define LOAD_PLUGIN_FUNCTION( rtype, implementationRef, Function, ... ) implementationRef -> Function = (rtype (*)( __VA_ARGS__ )) LOAD_PLUGIN_SYMBOL( pluginHandle, NAME_STRING( Function ) );

/// Automates declaration of all exported symbol library Functiontions from @a INTERFACE interface definition    
#define DECLARE_MODULE_INTERFACE( INTERFACE ) INTERFACE( Module, DECLARE_MODULE_FUNCTION )
/// Automates declaration of all Functiontion pointers from @a INTERFACE interface definition
#define DECLARE_MODULE_INTERFACE_REF( INTERFACE ) INTERFACE( Module, DECLARE_MODULE_FUNC_REF )
/// Automates definition of all Functiontion pointers from @a INTERFACE interface definition
#define DEFINE_MODULE_INTERFACE_REF( INTERFACE ) INTERFACE( Module, DEFINE_MODULE_FUNC_REF )

/// Automates definition of dummy implementations for all Functiontions from @a INTERFACE interface definition
#define DEFINE_MODULE_INTERFACE_STUB( INTERFACE ) INTERFACE( Module, DEFINE_MODULE_FUNC_STUB )

/// Automates loading of ibrary Functiontion symbols defined in @a INTERFACE definition and their attribuition to correspondent interface struct Functiontion fields 
#define LOAD_PLUGIN_FUNCTIONS( INTERFACE, Module ) INTERFACE( Module, LOAD_PLUGIN_FUNCTION )
      
/// Automates loading of dynamic library from @a pluginPath and of its exported Functiontion symbols defined in @a INTERFACE interface definition (sets @a success to false on error)
#define LOAD_MODULE_IMPLEMENTATION( INTERFACE, pluginPath, Module, success ) \
  do { char pluginPathExt[ 256 ]; \
  sprintf( pluginPathExt, "%s." PLUGIN_EXTENSION, pluginPath ); \
  printf( "\ttrying to load plugin %s\n", pluginPathExt ); \
  PLUGIN_HANDLE pluginHandle = LOAD_PLUGIN( pluginPathExt ); \
  printf( "\tplugin handle: %p\n", pluginHandle ); \
  *(success) = (bool) pluginHandle; \
  LOAD_PLUGIN_FUNCTIONS( INTERFACE, (Module) ) } while( 0 )


#endif  // MODULES_H
