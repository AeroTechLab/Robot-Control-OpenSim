#include "nms_processor-nn.h"

#include "perceptron/multi_layer_perceptron.h"

NMSProcessor::NMSProcessor( OpenSim::Model& model, ActuatorsList& actuatorsList, const size_t samplesNumber ) 
: NMSProcessorBase( 2, samplesNumber )
{
  inputsNumber = model.getMuscles().getSize() + NMS_INPUT_VARS_NUMBER * actuatorsList.size();
  outputsNumber = NMS_OUTPUT_VARS_NUMBER * actuatorsList.size();
  
  SimTK::Vector initialParametersList = GetInitialParameters();
  SimTK::Vector parametersMinList( initialParametersList.size() ), parametersMaxList( initialParametersList.size() );
  for( int parameterIndex = 0; parameterIndex < initialParametersList.size(); parameterIndex++ )
  {
    parametersMinList[ parameterIndex ] = 0.5 * initialParametersList[ parameterIndex ];
    parametersMaxList[ parameterIndex ] = 1.5 * initialParametersList[ parameterIndex ];
  }
  setParameterLimits( parametersMinList, parametersMaxList );

  //DataLogging.SetBaseDirectory( "test" );
  //optimizationLog = DataLogging.InitLog( "joints/optimization", 6 );
}

NMSProcessor::~NMSProcessor()
{
  ResetSamplesStorage();

  //DataLogging.EndLog( optimizationLog );
}

SimTK::Vector NMSProcessor::GetInitialParameters()
{
  SimTK::Vector initialParametersList( 2 );
  initialParametersList[ 0 ] = 10;
  initialParametersList[ 1 ] = MAX_SAMPLES_COUNT / 2;

  return initialParametersList;
}

void NMSProcessor::SetParameters( const SimTK::Vector& parametersList )
{
  size_t hiddenNeuronsNumber = (size_t) parametersList[ 0 ];
  size_t trainingSamplesNumber = (size_t) parametersList[ 1 ];
  
  perceptron = MLPerceptron_InitNetwork( inputsNumber, outputsNumber, hiddenNeuronsNumber );
  
  SimTK::Array_<const double*> trainingInputsTable, trainingOutputsTable;
  for( size_t sampleIndex = 0; sampleIndex < trainingSamplesNumber; sampleIndex++ )
  {
    trainingInputsTable.push_back( inputSamplesList[ sampleIndex ].getContiguousScalarData() );
    trainingOutputsTable.push_back( outputSamplesList[ sampleIndex ].getContiguousScalarData() );
  }
  
  (void) MLPerceptron_Train( perceptron, trainingInputsTable.data(), trainingOutputsTable.data(), trainingSamplesNumber );
}

int NMSProcessor::objectiveFunc( const SimTK::Vector& parametersList, bool newCoefficients, SimTK::Real& remainingError ) const
{
  size_t hiddenNeuronsNumber = (size_t) parametersList[ 0 ];
  size_t trainingSamplesNumber = (size_t) parametersList[ 1 ];
  
  MLPerceptron testMLP = MLPerceptron_InitNetwork( inputsNumber, outputsNumber, hiddenNeuronsNumber );
  
  SimTK::Array_<const double*> trainingInputsTable, trainingOutputsTable;
  for( size_t sampleIndex = 0; sampleIndex < trainingSamplesNumber; sampleIndex++ )
  {
    trainingInputsTable.push_back( inputSamplesList[ sampleIndex ].getContiguousScalarData() );
    trainingOutputsTable.push_back( outputSamplesList[ sampleIndex ].getContiguousScalarData() );
  }
  
  double trainingError = MLPerceptron_Train( testMLP, trainingInputsTable.data(), trainingOutputsTable.data(), trainingSamplesNumber );
  
  trainingInputsTable.clear();
  trainingOutputsTable.clear();
  
  SimTK::Array_<const double*> validationInputsTable, validationOutputsTable;
  size_t validationSamplesNumber = 0;
  for( size_t sampleIndex = trainingSamplesNumber; sampleIndex < inputSamplesList.size(); sampleIndex++ )
  {
    validationInputsTable.push_back( inputSamplesList[ sampleIndex ].getContiguousScalarData() );
    validationOutputsTable.push_back( outputSamplesList[ sampleIndex ].getContiguousScalarData() );
    validationSamplesNumber++;
  }
  
  double validationError = MLPerceptron_Validate( testMLP, validationInputsTable.data(), validationOutputsTable.data(), validationSamplesNumber );

  validationInputsTable.clear();
  validationOutputsTable.clear();
    
	// Data logging for joint 0
// 	for( int sampleIndex = 0; sampleIndex < NMS_POS_VARS_NUMBER; sampleIndex++ )
// 	  DataLogging.RegisterValues( optimizationLog, 1, positionSample[ sampleIndex ] );
// 	DataLogging.RegisterValues( optimizationLog, 1, torqueSample[ NMS_FORCE_VARS_NUMBER + NMS_TORQUE_EXT ] );
// 	for( int sampleIndex = 0; sampleIndex < emgSample.size(); sampleIndex++ )
// 	  DataLogging.RegisterValues( optimizationLog, 1, emgSample[ sampleIndex ] );
// 	DataLogging.RegisterValues( optimizationLog, 2, torqueSample[ NMS_FORCE_VARS_NUMBER + NMS_TORQUE_EXT ], emgTorqueOutputs[ NMS_FORCE_VARS_NUMBER + NMS_TORQUE_INT ] );
// 	DataLogging.RegisterValues( optimizationLog, 2, torqueSample[ NMS_FORCE_VARS_NUMBER + NMS_TORQUE_EXT ], emgTorqueOutputs[ NMS_FORCE_VARS_NUMBER + NMS_STIFFNESS ] );
// 	DataLogging.EnterNewLine( optimizationLog );

  remainingError = trainingError + 0.5 * validationError;
    
  std::cout << "objective function error: " << remainingError << std::endl;

  return 0;
}

SimTK::Vector NMSProcessor::CalculateOutputs( const SimTK::Vector& dynInputs, const SimTK::Vector& emgInputs ) const
{
  double* inputsList = new double[ inputsNumber ];
  double* outputsList = new double[ outputsNumber ];
  
  for( size_t valueIndex = 0; valueIndex < dynInputs.size(); valueIndex++ )
    inputsList[ valueIndex ] = dynInputs[ valueIndex ];
  for( size_t valueIndex = 0; valueIndex < emgInputs.size(); valueIndex++ )
    inputsList[ dynInputs.size() + valueIndex ] = emgInputs[ valueIndex ];
  
  MLPerceptron_ProcessInput( perceptron, inputsList, outputsList );
  
  SimTK::Vector torqueInternalOutputs( outputsNumber, outputsList );
  
  delete[] outputsList;
  
  return torqueInternalOutputs;
}
