#ifndef MULTI_LAYER_PERCEPTRONS_H
#define MULTI_LAYER_PERCEPTRONS_H

#include "namespaces.h"

typedef struct _MLPerceptronData MLPerceptronData;
typedef MLPerceptronData* MLPerceptron;

#define MULTI_LAYER_PERCEPTRON_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( MLPerceptron, Namespace, InitNetwork, size_t, size_t, size_t ) \
		INIT_FUNCTION( void, Namespace, EndNetwork, MLPerceptron ) \
		INIT_FUNCTION( double, Namespace, Train, MLPerceptron, double**, double**, size_t ) \
		INIT_FUNCTION( void, Namespace, ProcessInput, MLPerceptron, double*, double* )

DECLARE_NAMESPACE_INTERFACE( MLPerceptrons, MULTI_LAYER_PERCEPTRON_INTERFACE );

#endif // MULTI_LAYER_PERCEPTRONS_H