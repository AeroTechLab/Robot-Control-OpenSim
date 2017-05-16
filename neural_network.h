#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H

#include "namespaces.h"

typedef struct _NeuralNetworkData NeuralNetworkData;
typedef NeuralNetworkData* NeuralNetwork;

#define NEURAL_NETWORK_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( NeuralNetwork, Namespace, InitNetwork, size_t, size_t, size_t ) \
		INIT_FUNCTION( void, Namespace, EndNetwork, NeuralNetwork ) \
		INIT_FUNCTION( void, Namespace, EnterTrainingData, NeuralNetwork, double**, double**, size_t ) \
		INIT_FUNCTION( void, Namespace, ProcessInput, NeuralNetwork, double*, double* )

DECLARE_NAMESPACE_INTERFACE( NeuralNetworks, NEURAL_NETWORK_INTERFACE );

#endif // NEURAL_NETWORK_H