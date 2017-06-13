#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "multi_layer_perceptron.h"

enum { LIMIT_MIN, LIMIT_MAX, LIMITS_NUMBER };
typedef double Limits[ LIMITS_NUMBER ];

struct _MLPerceptronData
{
  double* inputsList;						// x'
  double* hiddenOutputsList;				// y'
  double** inputWeightsTable;				// Wji
  double** outputWeightsTable;				// Wkj
  Limits* inputLimitsList;
  Limits* outputLimitsList;
  size_t inputsNumber, outputsNumber;
  size_t hiddenNeuronsNumber;
};


DEFINE_NAMESPACE_INTERFACE( MLPerceptrons, MULTI_LAYER_PERCEPTRON_INTERFACE );


MLPerceptron MLPerceptrons_InitNetwork( size_t inputsNumber, size_t outputsNumber, size_t hiddenNeuronsNumber )
{
  MLPerceptronData* newNetwork = (MLPerceptronData*) malloc( sizeof(MLPerceptronData) );
    
  // Allocate memory for network inputs and hidden neurons outputs (extra memory for -1 threshold/offset)
  newNetwork->inputsList = (double*) calloc( inputsNumber + 1, sizeof(double) );
  newNetwork->hiddenOutputsList = (double*) calloc( hiddenNeuronsNumber + 1, sizeof(double) );
  // Allocate memory for input weights (extra memory for threshold/offset "weight" applied to -1 input)
  newNetwork->inputWeightsTable = (double**) calloc( hiddenNeuronsNumber, sizeof(double*) );
  for( size_t neuronIndex = 0; neuronIndex < hiddenNeuronsNumber; neuronIndex++ )
	newNetwork->inputWeightsTable[ neuronIndex ] = (double*) calloc( inputsNumber + 1, sizeof(double) );
  // Allocate memory for output weights (extra memory for threshold/offset "weight" applied to -1 hidden "output")
  newNetwork->outputWeightsTable = (double**) calloc( outputsNumber, sizeof(double*) );
  for( size_t neuronIndex = 0; neuronIndex < outputsNumber; neuronIndex++ )
	newNetwork->outputWeightsTable[ neuronIndex ] = (double*) calloc( hiddenNeuronsNumber + 1, sizeof(double) );
  // Allocate memory for network inputs/outputs maximum and minimum values
  newNetwork->inputLimitsList = (Limits*) calloc( inputsNumber, sizeof(Limits) );
  newNetwork->outputLimitsList = (Limits*) calloc( outputsNumber, sizeof(Limits) );

  newNetwork->inputsNumber = inputsNumber;
  newNetwork->outputsNumber = outputsNumber;
  newNetwork->hiddenNeuronsNumber = hiddenNeuronsNumber;

  return (MLPerceptron) newNetwork;
}

void MLPerceptrons_EndNetwork( MLPerceptron network )
{
  if( network == NULL ) return;

  free( network->hiddenOutputsList );

  for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber; neuronIndex++ )
	free( network->inputWeightsTable[ neuronIndex ] );
  free( network->inputWeightsTable );

  for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber; neuronIndex++ )
	free( network->outputWeightsTable[ neuronIndex ] );
  free( network->outputWeightsTable );

  free( network->inputLimitsList );
  free( network->outputLimitsList );
  
  free( network );
}

static void GetInputLimits( double** inputSamplesTable, size_t samplesNumber, size_t inputsNumber, Limits* inputLimitsList )
{
  memset( inputLimitsList, 0, inputsNumber * sizeof(Limits) );
  // Find min/max input values
  for( size_t sampleIndex = 0; sampleIndex < samplesNumber; sampleIndex++ )
  {
	double* inputSamplesList = inputSamplesTable[ sampleIndex ];
	for( size_t inputIndex = 0; inputIndex < inputsNumber; inputIndex++ )
	{
	  if( inputSamplesList[ inputIndex ] < inputLimitsList[ inputIndex ][ LIMIT_MIN ] ) inputLimitsList[ inputIndex ][ LIMIT_MIN ] = inputSamplesList[ inputIndex ];
	  if( inputSamplesList[ inputIndex ] > inputLimitsList[ inputIndex ][ LIMIT_MAX ] ) inputLimitsList[ inputIndex ][ LIMIT_MIN ] = inputSamplesList[ inputIndex ];
	}
  }
}

static void NormalizeInputs( double* inputsList, size_t inputsNumber, Limits* inputLimitsList )
{
  for( size_t inputIndex = 0; inputIndex < inputsNumber; inputIndex++ )
  {
	double inputRange = inputLimitsList[ inputIndex ][ LIMIT_MAX ] - inputLimitsList[ inputIndex ][ LIMIT_MIN ];
	inputsList[ inputIndex ] = ( inputsList[ inputIndex ] - inputLimitsList[ inputIndex ][ LIMIT_MIN ] ) / inputRange;
  }
}

static void GetOutputLimits( double** outputSamplesTable, size_t samplesNumber, size_t outputsNumber, Limits* outputLimitsList )
{
  memset( outputLimitsList, 0, outputsNumber * sizeof(Limits) );
  // Find min/max output values
  for( size_t sampleIndex = 0; sampleIndex < samplesNumber; sampleIndex++ )
  {
	double* outputSamplesList = outputSamplesTable[ sampleIndex ];
	for( size_t outputIndex = 0; outputIndex < outputsNumber; outputIndex++ )
	{
	  if( outputSamplesList[ outputIndex ] < outputLimitsList[ outputIndex ][ LIMIT_MIN ] ) outputLimitsList[ outputIndex ][ LIMIT_MIN ] = outputSamplesList[ outputIndex ];
	  if( outputSamplesList[ outputIndex ] > outputLimitsList[ outputIndex ][ LIMIT_MAX ] ) outputLimitsList[ outputIndex ][ LIMIT_MIN ] = outputSamplesList[ outputIndex ];
	}
  }
}

static void DenormalizeOutputs( double* outputsList, size_t outputsNumber, Limits* outputLimitsList )
{
  for( size_t outputIndex = 0; outputIndex < outputsNumber; outputIndex++ )
  {
	double outputRange = outputLimitsList[ outputIndex ][ LIMIT_MAX ] - outputLimitsList[ outputIndex ][ LIMIT_MIN ];
	outputsList[ outputIndex ] = outputsList[ outputIndex ] * outputRange + outputLimitsList[ outputIndex ][ LIMIT_MIN ];
  }
}

double MLPerceptrons_Train( MLPerceptron network, double** inputSamplesTable, double** outputSamplesTable, size_t samplesNumber )
{
  const size_t MAX_EPOCHS_NUMBER = 10;
  const double TOLERANCE = 1e-6;
  const double LEARNING_RATE = 0.4, LEARNING_MOMENT = 0.1;

  // Initialize sequence o sample indexes for random selection during training
  size_t* randomIndexesList = (size_t*) calloc( samplesNumber, sizeof(size_t) );
  for (size_t sampleIndex = 0; sampleIndex < samplesNumber; sampleIndex++)
	randomIndexesList[ sampleIndex ] = sampleIndex;

  double* hiddenActivationsList = (double*) calloc( network->hiddenNeuronsNumber, sizeof(double) );		// Hidden layer internal neuron activations
  double* outputActivationsList = (double*) calloc( network->outputsNumber, sizeof(double) );			// Output neurons activations
  double* outputGradientsList = (double*) calloc( network->outputsNumber, sizeof(double) );				// Output neurons output gradients
  double* fittingErrorsList = (double*) calloc( network->outputsNumber, sizeof(double) );				// Deviation between current epochCount outputs and reference samples
  double epochError = 0.0;																				// Average epoch deviation from samples (square error)

  //srand(time(NULL));

  // Random initialization of hidden layer and output layer weights
  for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber; neuronIndex++ )
  {
	for( size_t inputIndex = 0; inputIndex < network->inputsNumber; inputIndex++ )
	  network->inputWeightsTable[ neuronIndex ][ inputIndex ] = (rand() % 1000) / 1000.0;
  }
  for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber; neuronIndex++ )
  {
	for( size_t outputIndex = 0; outputIndex < network->outputsNumber; outputIndex++ )
	  network->outputWeightsTable[ neuronIndex ][ outputIndex ] = (rand() % 1000) / 1000.0;
  }

  GetInputLimits( inputSamplesTable, samplesNumber, network->inputsNumber, network->inputLimitsList );
  GetOutputLimits( outputSamplesTable, samplesNumber, network->outputsNumber, network->outputLimitsList );

  // Adjust weights (train) the network until error is below tolerance or maximum number of epochs is reached
  for( size_t epochCount = 0; epochCount < MAX_EPOCHS_NUMBER; epochCount++ ) 
  {
	// Permutation of training sample indexes (to avoid overfitting) 
	for( size_t sampleIndex = 0; sampleIndex < samplesNumber; sampleIndex++ )
	{
	  size_t nextRandomIndex = rand() % samplesNumber;
	  size_t currentRandomIndex = randomIndexesList[ sampleIndex ];
	  randomIndexesList[ sampleIndex ] = randomIndexesList[ nextRandomIndex ];
	  randomIndexesList[ nextRandomIndex ] = currentRandomIndex;
	}

	// Adjust weights and sum resulting square error for all samples (reference input/output sets) in random order
	for( size_t sampleIndex = 0; sampleIndex < samplesNumber; sampleIndex++ )
	{
	  // Pick next input/output training sample from the shuffled indexes list
	  size_t randomIndex = randomIndexesList[ sampleIndex ];          
	  double* inputSamplesList = inputSamplesTable[ randomIndex ];
	  double* outputSamplesList = outputSamplesTable[ randomIndex ];

	  // Normalize and add -1 input at the end of inputs list
	  memcpy( network->inputsList, inputSamplesList, network->inputsNumber * sizeof(double) );
	  NormalizeInputs( network->inputsList, network->inputsNumber, network->inputLimitsList );
	  network->inputsList[ network->inputsNumber ] = -1.0;

	  // First/hidden layer: calculate hidden neurons internal activations and outputs
	  for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber; neuronIndex++ ) 
	  {
		// Hidden neuron internal activation: u = Wji * x - theta1 = Wji' * x'
		hiddenActivationsList[ neuronIndex ] = 0.0;
		for( size_t inputIndex = 0; inputIndex < network->inputsNumber + 1; inputIndex++ )
		  hiddenActivationsList[ neuronIndex ] += network->inputWeightsTable[ neuronIndex ][ inputIndex ] * network->inputsList[ inputIndex ];
		// Hidden layer/neuron output (activation function): y' = sigmoid(u)
		network->hiddenOutputsList[ neuronIndex ] = 1.0 / ( 1.0 + exp( -hiddenActivationsList[ neuronIndex ] ) );
	  }
	  network->hiddenOutputsList[ network->hiddenNeuronsNumber ] = -1.0;	// Add threshold input

	  // Second/output layer: calculate network outputs
	  for( size_t outputIndex = 0; outputIndex < network->outputsNumber; outputIndex++ )
	  {
		// Output neuron internal activation: u' = Wkj * y' - theta2 = Wkj' * y'
		outputActivationsList[ outputIndex ] = 0.0;
		for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber + 1; neuronIndex++ )
		  outputActivationsList[ outputIndex ] += network->outputWeightsTable[ neuronIndex ][ outputIndex ] * network->hiddenOutputsList[ neuronIndex ];
        // Exit/output neuron output (activation function): y = sigmoid(u')
		double output = 1.0 / ( 1.0 + exp( -outputActivationsList[ outputIndex ] ) );

		// Current/epoch network output error and square error sum
		fittingErrorsList[ outputIndex ] = outputSamplesList[ outputIndex ] - output;
		epochError += pow( fittingErrorsList[ outputIndex ], 2 );
	  }

	  // Backpropagation traning: output layer
	  for( size_t outputIndex = 0; outputIndex < network->outputsNumber; outputIndex++ )
	  {
		double outputDerivative = exp( -outputActivationsList[ outputIndex ] ) / pow( 1.0 + exp( -outputActivationsList[ outputIndex ] ), 2 );	// sigmoid function derivative at output activation 
		outputGradientsList[ outputIndex ] = fittingErrorsList[ outputIndex ] * outputDerivative;										// output weight gradient: grad = output_error * output_derivative
		// hidden_output_weight = hidden_output_weight + LEARNING_RATE * grad * hidden_output
		for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber + 1; neuronIndex++ )
		  network->outputWeightsTable[ neuronIndex ][ outputIndex ] += LEARNING_RATE * outputGradientsList[ outputIndex ] * network->hiddenOutputsList[ neuronIndex ];
	  }

	  // Backpropagation traning: hidden layer
	  for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber; neuronIndex++ ) 
	  {
		// Local gradient calculation: grad = sum( output_delta * output_weight ) * hidden_output_derivative
		double hiddenOutputGradient = 0.0;
		for( size_t outputIndex = 0; outputIndex < network->outputsNumber; outputIndex++ )
		  hiddenOutputGradient += outputGradientsList[ outputIndex ] * network->outputWeightsTable[ network->hiddenNeuronsNumber ][ outputIndex ];
		hiddenOutputGradient *= exp( -hiddenActivationsList[ neuronIndex ] ) / pow( 1.0 + exp( -hiddenActivationsList[ neuronIndex ] ), 2 );		// sigmoid function derivative at hidden activation
	    // weight = weight + LEARNING_RATE * grad * input
		for( size_t inputIndex = 0; inputIndex < network->inputsNumber + 1; inputIndex++ )
		  network->inputWeightsTable[ neuronIndex ][ inputIndex ] += LEARNING_RATE * hiddenOutputGradient * network->inputsList[ inputIndex ];
	  }
	}

	epochError = epochError / samplesNumber;

	if( epochError < TOLERANCE ) break;
  }

  free( randomIndexesList );
  free( hiddenActivationsList );
  free( outputActivationsList );
  free( outputGradientsList );
  free( fittingErrorsList );

  return epochError;
}

void MLPerceptrons_ProcessInput( MLPerceptron network, double* inputsList, double* outputsList )
{
  if( network == NULL ) return;

  // Normalize and add -1 input at the end of inputs list
  memcpy( network->inputsList, inputsList, network->inputsNumber * sizeof(double) );
  NormalizeInputs( network->inputsList, network->inputsNumber, network->inputLimitsList );
  network->inputsList[ network->inputsNumber ] = -1.0;

  // First/hidden layer: calculate hidden neurons internal activations and outputs
  for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber; neuronIndex++ ) 
  {
	// Hidden neuron internal activation: u = Wji * x - theta1 = Wji' * x'
	network->hiddenOutputsList[ neuronIndex ] = 0.0;
	for( size_t inputIndex = 0; inputIndex < network->inputsNumber + 1; inputIndex++ )
	  network->hiddenOutputsList[ neuronIndex ] += network->inputWeightsTable[ neuronIndex ][ inputIndex ] * network->inputsList[ inputIndex ];
	// Hidden layer/neuron output (activation function): y' = sigmoid(u)
	network->hiddenOutputsList[ neuronIndex ] = 1.0 / ( 1.0 + exp( -network->hiddenOutputsList[ neuronIndex ] ) );
  }
  network->hiddenOutputsList[ network->hiddenNeuronsNumber ] = -1.0;	// Add threshold input

  // Second/output layer: calculate network outputs
  for( size_t outputIndex = 0; outputIndex < network->outputsNumber; outputIndex++ )
  {
	// Output neuron internal activation: u' = Wkj * y' - theta2 = Wkj' * y'
	outputsList[ outputIndex ] = 0.0;
	for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber + 1; neuronIndex++ )
	  outputsList[ outputIndex ] += network->outputWeightsTable[ neuronIndex ][ outputIndex ] * network->hiddenOutputsList[ neuronIndex ];
    // Exit/output neuron output (activation function): y = sigmoid(u')
	outputsList[ outputIndex ] = 1.0 / ( 1.0 + exp( -outputsList[ outputIndex ] ) );
  }

  DenormalizeOutputs( outputsList, network->outputsNumber, network->outputLimitsList );
} 
