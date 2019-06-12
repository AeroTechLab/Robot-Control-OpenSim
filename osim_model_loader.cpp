#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <simbody/internal/Visualizer_InputListener.h>

#include <iostream>
#include <chrono>

class SliderEventHandler : public SimTK::PeriodicEventHandler
{
public:
  SliderEventHandler( SimTK::Visualizer::InputSilo& inputSilo, int& sliderID, OpenSim::CoordinateActuator* actuator, SimTK::Real updateInterval )
  : PeriodicEventHandler( updateInterval ), sliderID( sliderID ), inputSilo( inputSilo )
  {
    this->actuator = actuator;
  }
  
  void handleEvent( SimTK::State& state, SimTK::Real accuracy, bool &shouldTerminate ) const
  {
    while( inputSilo.isAnyUserInput() ) 
    {
      std::cout << "received input" << std::endl;
      SimTK::Real userInputValue;
      while( inputSilo.takeSliderMove( sliderID, userInputValue ) ) 
        actuator->setOverrideActuation( state, userInputValue );
    }
  }
  
private:
  int& sliderID;
  SimTK::Visualizer::InputSilo& inputSilo;
  OpenSim::CoordinateActuator* actuator;
};

int main( int argc, char* argv[] )
{ 
  const char* REFERENCE_AXIS_NAMES[ 3 ] = { "_x", "_y", "_z" };
  
  std::vector<OpenSim::CoordinateActuator*> actuatorsList;
  std::vector<int> accelerationIndexesList;
  SimTK::Array_<OpenSim::CoordinateReference> coordinateReferences;
  OpenSim::MarkerSet markers;
  OpenSim::MarkersReference markersReference;
  OpenSim::Set<OpenSim::MarkerWeight> markerWeights;
  std::vector<char*> jointNamesList;
  std::vector<char*> axisNamesList;
  std::vector<std::string> markerLabels;
  
  if( argc < 2 ) exit( -1 );
  
  try 
  { // Create an OpenSim model from XML (.osim) file
    OpenSim::Model osimModel( argv[ 1 ] );
    osimModel.printBasicInfo( std::cout );    
    osimModel.setGravity( SimTK::Vec3( 0.0, -9.80665, 0.0 ) );
    osimModel.setUseVisualizer( true ); // not for RT
    OpenSim::MarkerSet& markerSet = osimModel.updMarkerSet(); 
    std::cout << "OSim: found " << markerSet.getSize() << " markers" << std::endl;
    for( int markerIndex = 0; markerIndex < markerSet.getSize(); markerIndex++ )
    {
      std::string markerName = markerSet[ markerIndex ].getName();
      if( markerName.find( "_ref" ) != std::string::npos )
      {
        std::cout << "OSim: found reference marker " << markerName << std::endl;
        markerWeights.adoptAndAppend( new OpenSim::MarkerWeight( markerName, 1.0 ) );
        markers.adoptAndAppend( &(markerSet[ markerIndex ]) );
        markerLabels.push_back( markerName );
        for( size_t referenceAxisIndex = 0; referenceAxisIndex < 3; referenceAxisIndex++ )
        {
          std::string markerAxisName = markerName + REFERENCE_AXIS_NAMES[ referenceAxisIndex ];
          axisNamesList.push_back( (char*) markerAxisName.c_str() );
        }
      }
    }
    std::cout << "OSim: generated markers reference size" << markersReference.updMarkerWeightSet().getSize() << std::endl;
    std::cout << "OSim: generated coordinates reference size: " << coordinateReferences.size() << std::endl;
    const OpenSim::Set<OpenSim::Muscle> muscleSet = osimModel.getMuscles();
    const OpenSim::Set<OpenSim::Actuator>& actuatorSet = osimModel.getActuators();
    osimModel.buildSystem();
    for( int actuatorIndex = 0; actuatorIndex < actuatorSet.getSize(); actuatorIndex++ )
    {
      std::string actuatorName = actuatorSet[ actuatorIndex ].getName();
      if( not muscleSet.contains( actuatorName ) )
      {
        OpenSim::CoordinateActuator* actuator = dynamic_cast<OpenSim::CoordinateActuator*>(&(actuatorSet[ actuatorIndex ]));
        if( actuator != NULL )
        {
          std::cout << "Found coordinate actuator " << actuator->getName() << std::endl;
          std::cout << "Actuator coordinate: " << actuator->getCoordinate()->getName() << std::endl;
          //coordinateReferences.push_back( OpenSim::CoordinateReference( actuator->getCoordinate()->getName(), OpenSim::Constant( 0.0 ) ) );
          actuatorsList.push_back( actuator );
          jointNamesList.push_back( (char*) actuator->getCoordinate()->getName().c_str() );
          accelerationIndexesList.push_back( osimModel.getCoordinateSet().getIndex( actuator->getCoordinate()->getName() ) );
        }
      }
    }
    std::cout << "OpenSim model loaded successfully ! (" << osimModel.getNumCoordinates() << " coordinates)" << std::endl;
    SimTK::State& state = osimModel.initializeState();    // Initialize the system
    for( int muscleIndex = 0; muscleIndex < muscleSet.getSize(); muscleIndex++ )
#ifdef OSIM_LEGACY
      muscleSet[ muscleIndex ].setDisabled( state, true );
#else
      muscleSet[ muscleIndex ].setAppliesForce( state, false );
#endif
    for( size_t jointIndex = 0; jointIndex < actuatorsList.size(); jointIndex++ )
    {
#ifdef OSIM_LEGACY
      actuatorsList[ jointIndex ]->overrideForce( state, true );
#else
      actuatorsList[ jointIndex ]->overrideActuation( state, true );
#endif
      actuatorsList[ jointIndex ]->getCoordinate()->setValue( state, 0.0 );
    }
    std::vector<SimTK::Vec3> markerSetpoints( markers.getSize(), SimTK::Vec3( 0.0 ) );
    std::vector<SimTK::Vec3> markerInitialLocations( markers.getSize(), SimTK::Vec3( 0.0 ) );
    for( int markerIndex = 0; markerIndex < markers.getSize(); markerIndex++ )
    {
      for( size_t axisIndex = 0; axisIndex < 3; axisIndex++ )
      {
        int sliderID = 3 * markerIndex + axisIndex;
        std::string axisLabel = markerLabels[ markerIndex ] + REFERENCE_AXIS_NAMES[ axisIndex ];
        osimModel.updVisualizer().updSimbodyVisualizer().addSlider( axisLabel, sliderID, -1.0, 1.0, 0.0 );
      }
#ifdef OSIM_LEGACY
      osimModel.getSimbodyEngine().getPosition( state, markers[ markerIndex ].getBody(), markers[ markerIndex ].getOffset(), markerInitialLocations[ markerIndex ] );
#else
      markerInitialLocations[ markerIndex ] = markers[ markerIndex ].getLocationInGround( state );
#endif
    }
    state.setTime( 0.0 );
    SimTK::State ikState = state;
    std::chrono::steady_clock::time_point initialTime = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point simulationTime = std::chrono::steady_clock::now();
    SimTK::Matrix_<SimTK::Vec3> markersTable( 1, markers.getSize() );
    while( true )
    {
      simulationTime = std::chrono::steady_clock::now();
      while( osimModel.updVisualizer().updInputSilo().isAnyUserInput() ) 
      {
        int sliderID;
        SimTK::Real userInputValue;
        while( osimModel.updVisualizer().updInputSilo().takeSliderMove( sliderID, userInputValue ) )
        {
          //std::cout << "received input on slider " << sliderID << ", marker " << sliderID / 3 << ", axis " << sliderID % 3 << std::endl;
          size_t markerIndex = (size_t) ( sliderID / 3 );
          size_t axisIndex = (size_t) ( sliderID % 3 );
          markerSetpoints[ markerIndex ][ axisIndex ] = userInputValue;
          markersTable.set( 0, markerIndex, markerInitialLocations[ markerIndex ] + markerSetpoints[ markerIndex ] );
          //std::cout << "setpoint ( " << markerSetpoints[ markerIndex ][ 0 ] << ", " << markerSetpoints[ markerIndex ][ 1 ] << ", " << markerSetpoints[ markerIndex ][ 2 ] << " )" << std::endl;
        }
      }
      OpenSim::TimeSeriesTableVec3 markersTimeTable( std::vector<double>( { ikState.getTime() } ), markersTable, markerLabels );
      OpenSim::MarkersReference markersReference( markersTimeTable, &markerWeights );
      OpenSim::InverseKinematicsSolver ikSolver( osimModel, markersReference, coordinateReferences, 0.0 );
      ikSolver.setAccuracy( 1.0e-4 ); //std::cout << "OSim: IK solver set up" << std::endl;
      ikSolver.assemble( ikState ); //std::cout << "OSim: IK solver assembled" << std::endl;
      ikSolver.track( ikState );
      std::cout << "simulation time: " << state.getTime();
      for( size_t jointIndex = 0; jointIndex < actuatorsList.size(); jointIndex++ )
      {
        OpenSim::Coordinate* jointCoordinate = actuatorsList[ jointIndex ]->getCoordinate();
        jointCoordinate->setValue( state, jointCoordinate->getValue( ikState ) );
        //jointCoordinate->setSpeedValue( state, jointCoordinate->getSpeedValue( ikState ) );
        //jointCoordinate->setAccelerationValue( state, jointCoordinate->getAccelerationValue( ikState ) );
        std::cout << ", position: " << jointCoordinate->getValue( state );
      }
      std::cout << std::endl;
      OpenSim::Manager manager( osimModel );
      manager.initialize( state );
      state = manager.integrate( std::chrono::duration_cast<std::chrono::duration<double>>( simulationTime - initialTime ).count() );
      std::this_thread::sleep_until( simulationTime + std::chrono::milliseconds( 10 ) );      
    }    
  }
  catch( OpenSim::Exception ex )
  {
    std::cout << ex.getMessage() << std::endl;
    exit( -1 );
  }
  catch( std::exception ex )
  {
    std::cout << ex.what() << std::endl;
    exit( -1 );
  }
  catch( ... )
  {
    std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
    exit( -1 );
  }
  
  exit( 0 );
}
