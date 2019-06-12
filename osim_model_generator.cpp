#include <OpenSim/OpenSim.h>

#include <iostream>

int main()
{
  const size_t BODIES_NUMBER = 2;
  const SimTK::Vec3 BODY_COLORS[] = { SimTK::Red, SimTK::Blue, SimTK::Green, SimTK::Yellow }; 
  const double BODY_SIZE = 0.5;
  const double BODY_DISTANCE = 3 * BODY_SIZE;
  std::cout << "creating osim model" << std::endl;
  try
  {
    OpenSim::Model osimModel;
    //osimModel.setUseVisualizer( true );
        
    osimModel.setName( "osim_robot" );
    osimModel.setGravity( SimTK::Vec3( 0, 0, 0 ) );

    for( size_t bodyIndex = 0; bodyIndex < BODIES_NUMBER; bodyIndex++ )
    {
      std::cout << "creating osim body" << std::endl;
      std::string indexString = std::to_string( bodyIndex );
      OpenSim::Body* body = new OpenSim::Body( "body_" + indexString, 1.0, SimTK::Vec3( 0, 0, 0 ), SimTK::Inertia( 1, 1, 1 ) );
      osimModel.addBody( body );
      
      const OpenSim::PhysicalFrame& refFrame = ( bodyIndex == 0 ) ? (const OpenSim::PhysicalFrame&) osimModel.getGround() : (const OpenSim::PhysicalFrame&) osimModel.getBodySet().get( 0 );
      OpenSim::PinJoint* pinJoint = new OpenSim::PinJoint( "joint_" + indexString, 
                                                          refFrame, SimTK::Vec3( 0, BODY_DISTANCE, 0 ), SimTK::Vec3( 0, 0, 0 ), 
                                                          *(body), SimTK::Vec3( 0, 0, 0 ), SimTK::Vec3( 0, 0, 0 ) );
      osimModel.addJoint( pinJoint );
      
      OpenSim::Cylinder* bodyMesh = new OpenSim::Cylinder( BODY_SIZE, BODY_SIZE );
      bodyMesh->setColor( BODY_COLORS[ bodyIndex ] );
      OpenSim::PhysicalOffsetFrame* offsetFrame = new OpenSim::PhysicalOffsetFrame();
      offsetFrame->setParentFrame( *(body) );
      offsetFrame->set_orientation( SimTK::Vec3( SimTK::Pi / 2, 0.0, 0.0 ) );
      offsetFrame->attachGeometry( bodyMesh );
      body->addComponent( offsetFrame );
      offsetFrame = new OpenSim::PhysicalOffsetFrame();
      offsetFrame->setParentFrame( *(body) );
      offsetFrame->set_translation( SimTK::Vec3( 0.0, BODY_DISTANCE / 2, 0.0 ) );
      offsetFrame->attachGeometry( new OpenSim::Brick( SimTK::Vec3( BODY_SIZE / 5, BODY_DISTANCE / 2, BODY_SIZE / 2 ) ) );
      body->addComponent( offsetFrame );
      offsetFrame = new OpenSim::PhysicalOffsetFrame();
      offsetFrame->setParentFrame( *(body) );
      offsetFrame->set_translation( SimTK::Vec3( 0.0, BODY_DISTANCE, 0.0 ) );
      offsetFrame->set_orientation( SimTK::Vec3( SimTK::Pi / 2, 0.0, 0.0 ) );
      bodyMesh = new OpenSim::Cylinder( BODY_SIZE / 2, BODY_SIZE );
      bodyMesh->setColor( BODY_COLORS[ bodyIndex ] );
      offsetFrame->attachGeometry( bodyMesh );
      body->addComponent( offsetFrame );
      
      if( bodyIndex > 0 )
      {
        OpenSim::Marker* effectorMarker = new OpenSim::Marker( "effector_ref", *body, SimTK::Vec3( 0.0, BODY_DISTANCE, 0.0 ) );
        osimModel.addMarker( effectorMarker );
      }
      
      OpenSim::Coordinate& coordinate = pinJoint->updCoordinate();
      OpenSim::CoordinateActuator* userActuator = new OpenSim::CoordinateActuator( coordinate.getName() );
      userActuator->setName( coordinate.getName() + "_user" );
      osimModel.addForce( userActuator );
      OpenSim::CoordinateActuator* controlActuator = new OpenSim::CoordinateActuator( coordinate.getName() );
      controlActuator->setName( coordinate.getName() + "_control" );
      osimModel.addForce( controlActuator );
    }
      
    SimTK::State& state = osimModel.initSystem();

    osimModel.print( "osim_robot.osim" );
  }
  catch( OpenSim::Exception exception )
  {
    std::cout << exception.getMessage() << std::endl;
    exit( -1 );
  }
  catch( std::exception exception )
  {
    std::cout << exception.what() << std::endl;
    exit( -1 );
  }
  catch( ... )
  {
    std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
    exit( -1 );
  }
  
  exit( 0 );
}
