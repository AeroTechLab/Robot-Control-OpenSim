#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "neural_network.h"

struct _NeuralNetworkData
{
  double* neuronActivationsList;
  double** inputWeightsTable;
  double** outputWeightsTable;
  size_t inputsNumber, outputsNumber;
  size_t hiddenNeuronsNumber;
};


DEFINE_NAMESPACE_INTERFACE( NeuralNetworks, NEURAL_NETWORK_INTERFACE );


NeuralNetwork NeuralNetworks_InitNetwork( size_t inputsNumber, size_t outputsNumber, size_t hiddenNeuronsNumber )
{
  NeuralNetworkData* newNetwork = (NeuralNetworkData*) malloc( sizeof(NeuralNetworkData) );
    
  newNetwork->neuronActivationsList = (double*) calloc( hiddenNeuronsNumber + 1, sizeof(double) );
  newNetwork->inputWeightsTable = (double**) calloc( hiddenNeuronsNumber, sizeof(double*) );
  newNetwork->outputWeightsTable = (double**) calloc( hiddenNeuronsNumber + 1, sizeof(double*) );
  for( size_t neuronIndex = 0; neuronIndex < hiddenNeuronsNumber; neuronIndex++ )
  {
	newNetwork->inputWeightsTable[ neuronIndex ] = (double*) calloc( inputsNumber + 1, sizeof(double) );
	newNetwork->outputWeightsTable[ neuronIndex ] = (double*) calloc( outputsNumber, sizeof(double) );
  }
  
  newNetwork->inputsNumber = inputsNumber;
  newNetwork->outputsNumber = outputsNumber;
  newNetwork->hiddenNeuronsNumber = hiddenNeuronsNumber;

  return (NeuralNetwork) newNetwork;
}

void NeuralNetworks_EndNetwork( NeuralNetwork network )
{
  if( network == NULL ) return;

  free( network->neuronActivationsList );

  for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber; neuronIndex++ )
	  free( network->inputWeightsTable[ neuronIndex ] );
  free( network->inputWeightsTable );

  for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber + 1; neuronIndex++ )
	  free( network->outputWeightsTable[ neuronIndex ] );
  free( network->outputWeightsTable );
  
  free( network );
}

void NeuralNetworks_EnterTrainingData( NeuralNetwork network, double** inputSamplesTable, double** outputSamplesTable, size_t samplesNumber )
{
  const size_t EPOCHS_NUMBER = 10;
  const double LEARNING_RATE = 0.4;

  size_t* randomIndexesList = (size_t*) calloc( samplesNumber, sizeof(size_t) );
  for (size_t sampleIndex = 0; sampleIndex < samplesNumber; sampleIndex++)
	randomIndexesList[ sampleIndex ] = sampleIndex;

  double* neuronValuesList = (double*) calloc( network->hiddenNeuronsNumber, sizeof(double) );

  double* outputsList = (double*) calloc( network->outputsNumber, sizeof(double) );
  double* fittingErrorsList = (double*) calloc( network->outputsNumber, sizeof(double) );
  double epochError = 0.0;

  //srand(time(NULL));

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

  for( size_t epoch = 0; epoch < EPOCHS_NUMBER; epoch++ ) 
  {
	  //e_total[epoca] = 0;
	  //y[0] = 0;

	  //Permutar entrada
	  // sorte = randperm(pt); 
	  for( size_t sampleIndex = 0; sampleIndex < samplesNumber; sampleIndex++ )
	  {
		  size_t nextRandomIndex = rand() % samplesNumber;
		  size_t currentRandomIndex = randomIndexesList[ sampleIndex ];
		  randomIndexesList[ sampleIndex ] = randomIndexesList[ nextRandomIndex ];
		  randomIndexesList[ nextRandomIndex ] = currentRandomIndex;
	  }

	  for( size_t sampleIndex = 0; sampleIndex < samplesNumber; sampleIndex++ )
	  {
		  size_t randomIndex = randomIndexesList[ sampleIndex ];

		  double* inputSamplesList = inputSamplesTable[ randomIndex ];
		  double* outputSamplesList = outputSamplesTable[ randomIndex ];

		  // MLP
		  for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber; neuronIndex++ ) 
		  {
			  neuronValuesList[ neuronIndex ] = 0.0;
			  for( size_t inputIndex = 0; inputIndex < network->inputsNumber; inputIndex++ )
				  neuronValuesList[ neuronIndex ] += inputSamplesList[ inputIndex ] * network->inputWeightsTable[ neuronIndex ][ inputIndex ];		/* Ativacao interna : Somatorio (XiWji) */
			  neuronValuesList[ neuronIndex ] -= network->inputWeightsTable[ neuronIndex ][ network->inputsNumber ];
			  network->neuronActivationsList[ neuronIndex ] = 1.0 / ( 1.0 + exp( -neuronValuesList[ neuronIndex ] ) );						/* Saida do neuronio j */
		  }
		  network->neuronActivationsList[ network->hiddenNeuronsNumber ] = -1.0;											/* Alimentacao para o Threshold */

																		/* Camada de saida - Forward */
		  for( size_t outputIndex = 0; outputIndex < network->outputsNumber; outputIndex++ )
		  {
			  outputsList[ outputIndex ] = 0.0;
			  for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber + 1; neuronIndex++ )
				  outputsList[ outputIndex ] += network->neuronActivationsList[ network->hiddenNeuronsNumber ] * network->inputWeightsTable[ neuronIndex ][ outputIndex ];		/* Ativacao interna : Somatorio (YhjWkj) */

			  double output = 1.0 / ( 1.0 + exp( -outputsList[ outputIndex ] ) );							/* Saida do neuronio k */

			  fittingErrorsList[ outputIndex ] = outputSamplesList[ outputIndex ] - output;
			  epochError += pow( fittingErrorsList[ outputIndex ], 2 );
		  }

		  // Treinamento utilizando retropropagação do erro
		  //for (k = 0; k < Neu_out; k++) {
			//  for (j = 0; j < Neu_hid + 1; j++)
			//	  Wkj_ant[k][j] = Wkj[k][j];		//Wkj_ant = Wkj;
		  //}
		  //for (j = 0; j < Neu_hid; j++) {
			//  for (i = 0; i < Neu_in + 1; i++)
			//	  Wji_ant[j][i] = Wji[j][i];		//Wji_ant = Wji;
		  //}

		  for( size_t outputIndex = 0; outputIndex < network->outputsNumber; outputIndex++ )
		  {
			  double outputDerivative = exp( -outputsList[ outputIndex ] ) / pow( 1.0 + exp( -outputsList[ outputIndex ] ), 2 );	  // derivada funcao sigmoidal
			  outputsList[ outputIndex ] = fittingErrorsList[ outputIndex ] * outputDerivative;	//deltak = deriv.*Ek
			  network->outputWeightsTable[ network->hiddenNeuronsNumber ][ outputIndex ] -= LEARNING_RATE * outputsList[ outputIndex ] /** network->neuronActivationsList[ network->hiddenNeuronsNumber ]*/;
			  //Wkj[k][Neu_hid + 1] = Wkj[k][Neu_hid + 1] 
				//                    + Momento*(Wkj_ant[k][Neu_hid + 1] - Wkj_ant2[k][Neu_hid + 1]) 
				//                    + Alfa*Delta_k[k]*Y_h[Neu_hid + 1];	// peso - threshold Wkj
		  }

		  for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber; neuronIndex++ ) 
		  {
			  double neuronWeightDelta = 0.0;
			  for( size_t outputIndex = 0; outputIndex < network->outputsNumber; outputIndex++ )
			  {
				  //Wkj[k][j] = Wkj[k][j] + Momento*(Wkj_ant[k][j] - Wkj_ant2[k][j]) + Alfa*Delta_k[k]*Y_h[j];
				  network->outputWeightsTable[ network->hiddenNeuronsNumber ][ outputIndex ];
				  neuronWeightDelta += outputsList[ outputIndex ] * network->outputWeightsTable[ network->hiddenNeuronsNumber ][ outputIndex ];	// somatorio(Deltak Wkj)
			  }
			  double activationDerivative = exp( -neuronValuesList[ neuronIndex ] ) / pow( 1.0 + exp( -neuronValuesList[ neuronIndex ] ), 2 );	 // derivada funcao sigmoidal
		      double inputWeightDelta = activationDerivative * neuronWeightDelta;						// deltaj = deriv.*somatorio(Wkj Deltak)
			  for( size_t inputIndex = 0; inputIndex < network->inputsNumber + 1; inputIndex++ )
				  network->inputWeightsTable[ neuronIndex ][ inputIndex ] += LEARNING_RATE * inputWeightDelta * inputSamplesList[ inputIndex ];
				  //Wji[j][i] = Wji[j][i] + Momento*(Wji_ant[j][i] - Wji_ant2[j][i]) + Alfa*Delta_j[j]*x[i];
			 
		  }
			  
		  //for (k = 0; k < Neu_out; k++) {
			//  for (j = 0; j < Neu_hid + 1; j++)
			//	  Wkj_ant2[k][j] = Wkj_ant[k][j];		//Wkj_ant2 = Wkj_ant;
		  //}
		  //for (j = 0; j < Neu_hid; j++) {
			//  for (i = 0; i < Neu_in + 1; i++)
			//	  Wji_ant2[j][i] = Wji_ant[j][i];		//Wji_ant2 = Wji_ant;
		  //} 
	  }

	  epochError = epochError / samplesNumber;

	 // printf("Wji = %f\n", Wji[0][0]);
	  //printf("Epoca= %u, Erro= %f\n", epoca, e_total[epoca]);

	  if( epochError < 0.0000001 ) break;
  }

  free( randomIndexesList );
  free( neuronValuesList );
  free( outputsList );
  free( fittingErrorsList );
}

void NeuralNetworks_ProcessInput( NeuralNetwork network, double* inputsList, double* outputsList )
{
  if( network == NULL ) return;

  // MLP
  /* Camada escondida - Forward */
  for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber; neuronIndex++ ) 
  {
	network->neuronActivationsList[ neuronIndex ] = 0.0;
	for( size_t inputIndex = 0; inputIndex < network->inputsNumber + 1; inputIndex++ )
	network->neuronActivationsList[ neuronIndex ] += inputsList[ inputIndex ] * network->inputWeightsTable[ neuronIndex ][ inputIndex ];	/* Ativacao interna : Somatorio (XiWji) */
	network->neuronActivationsList[ neuronIndex ] -= network->inputWeightsTable[ neuronIndex ][ network->inputsNumber ];
	network->neuronActivationsList[ neuronIndex ] = 1.0 / ( 1.0 + exp( -network->neuronActivationsList[ neuronIndex ] ) );				/* Saida do neuronio j */
  }
  network->neuronActivationsList[ network->hiddenNeuronsNumber ] = -1.0;											/* Alimentacao para o Threshold */

  /* Camada de saida - Forward */
  for( size_t outputIndex = 0; outputIndex < network->outputsNumber; outputIndex++ )  
  {
	outputsList[ outputIndex ] = 0.0;
	for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber + 1; neuronIndex++ )
	  outputsList[ outputIndex ] += network->neuronActivationsList[ network->hiddenNeuronsNumber ] * network->inputWeightsTable[ neuronIndex ][ outputIndex ];	/* Ativacao interna : Somatorio (YHjWkj) */
	outputsList[ outputIndex ] = 1.0 / ( 1.0 + exp( -outputsList[ outputIndex ] ) );							/* Saida do neuronio k */
  }
} 
