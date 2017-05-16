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


/// @file namespaces.h
/// @brief Helper macros for defining more Cpp-like scoped (namespaced) interfaces.
///
/// This set of macros allow for defining pseudo-namespaced functions that can be accessed
/// as rtype = Namespace.Method( args ), according to an interface definition like:
///
/// @code
/// #define MY_INTERFACE( Namespace, INIT_FUNCTION ) \
///     INIT_FUNCTION( int, Namespace, Method_1, void ) \
///     INIT_FUNCTION( void, Namespace, Method_2, int, char ) \
///     INIT_FUNCTION( char*, Namespace, Method_3, unsigned long )
/// @endcode

#ifndef NAMESPACES_H
#define NAMESPACES_H

#ifdef WIN32
  #ifdef DLL_EXPORT
    #define SYMBOL extern "C" __declspec(dllexport)
  #elif DLL_IMPORT
    #define SYMBOL extern "C" //__declspec(dllimport)
  #else
    #define SYMBOL extern
  #endif
#else
  #define SYMBOL extern
#endif

/// Automates declaration of internal (exposed by pointer) function signature
#define DECLARE_NAMESPACE_FUNCTION( rtype, Namespace, func, ... ) static rtype Namespace##_##func( __VA_ARGS__ );
/// Automates declaration of exposed and namespaced function pointer
#define DECLARE_NAMESPACE_FUNC_REF( rtype, Namespace, func, ... ) rtype (*func)( __VA_ARGS__ );        
/// Automates definition of namespaced method (internal function address attribuition to exposed function pointer)
#if __STDC_VERSION__ >= 199901L
#include <stdbool.h>
#define DEFINE_NAMESPACE_FUNC_REF( rtype, Namespace, func, ... ) .func = Namespace##_##func,
#else
#define DEFINE_NAMESPACE_FUNC_REF( rtype, Namespace, func, ... ) Namespace##_##func,
#define false   0
#define true    1
#define bool char
#endif


/// Automates declaration of namespace struct and all exposed and namespaced function pointers       
#define DECLARE_NAMESPACE_INTERFACE( Namespace, INTERFACE ) \
	    typedef struct { INTERFACE( Namespace, DECLARE_NAMESPACE_FUNC_REF ) } Namespace##_##Interface; \
        SYMBOL const Namespace##_##Interface Namespace
   
/// Automates declaration of all internal functions and definition of all exposed and namespaced function pointers            
#define DEFINE_NAMESPACE_INTERFACE( Namespace, INTERFACE ) \
        INTERFACE( Namespace, DECLARE_NAMESPACE_FUNCTION ) \
		const Namespace##_##Interface Namespace = { INTERFACE( Namespace, DEFINE_NAMESPACE_FUNC_REF ) }

#endif  // NAMESPACES_H
