import opensim

BODY_SIZE = 0.5
BODY_DISTANCE = 3 * BODY_SIZE

BLOCK_COLORS = [ opensim.Blue, opensim.Red ]

try:
  model = opensim.Model()
  
  model.setUseVisualizer( True )
  
  model.setName( 'Robot' )
  model.setGravity( opensim.Vec3( 0, 0, 0 ) )
  
  for index in range( 2 ):
    indexString = str( 1 )
    
    body = opensim.Body( "body_" + indexString, 1.0, opensim.Vec3( 0, 0, 0 ), opensim.Inertia( 1, 1, 1 ) )
    model.addBody( body )
    
    refFrame = model.getGround() if index == 0 else model.getBodySet().get( 0 )
    pinJoint = opensim.PinJoint( "joint_" + indexString, refFrame, opensim.Vec3( 0, BODY_DISTANCE, 0 ), opensim.Vec3( 0, 0, 0 ), body, opensim.Vec3( 0, 0, 0 ), opensim.Vec3( 0, 0, 0 ) )
    model.addJoint( pinJoint )

    bodyMesh = opensim.Cylinder( BODY_SIZE, BODY_SIZE )
    bodyMesh.setColor( BLOCK_COLORS[ index ] )
    offsetFrame = opensim.PhysicalOffsetFrame()
    offsetFrame.setParentFrame( body )
    offsetFrame.set_orientation( opensim.Vec3( opensim.SimTK_PI / 2, 0.0, 0.0 ) )
    offsetFrame.attachGeometry( bodyMesh )
    body.addComponent( offsetFrame )
    offsetFrame = opensim.PhysicalOffsetFrame()
    offsetFrame.setParentFrame( body )
    offsetFrame.set_translation( opensim.Vec3( 0.0, BODY_DISTANCE / 2, 0.0 ) )
    offsetFrame.attachGeometry( opensim.Brick( opensim.Vec3( BODY_SIZE / 5, BODY_DISTANCE / 2, BODY_SIZE / 2 ) ) )
    body.addComponent( offsetFrame )
    offsetFrame = opensim.PhysicalOffsetFrame()
    offsetFrame.setParentFrame( body )
    offsetFrame.set_translation( opensim.Vec3( 0.0, BODY_DISTANCE, 0.0 ) )
    offsetFrame.set_orientation( opensim.Vec3( opensim.SimTK_PI / 2, 0.0, 0.0 ) )
    bodyMesh = opensim.Cylinder( BODY_SIZE / 2, BODY_SIZE )
    bodyMesh.setColor( BLOCK_COLORS[ index ] )
    offsetFrame.attachGeometry( bodyMesh )
    body.addComponent( offsetFrame )
    
    if index == 1:
      markerX = opensim.Marker( 'effector_x', body, opensim.Vec3( 0, BODY_DISTANCE, 0 ) )
      model.addMarker( markerX )
      markerY = opensim.Marker( 'effector_y', body, opensim.Vec3( 0, BODY_DISTANCE, 0 ) )
      model.addMarker( markerY )
    
    coordinate = pinJoint.getCoordinate()
    userActuator = opensim.CoordinateActuator( coordinate.getName() )
    model.addForce( userActuator )
    controlActuator = opensim.CoordinateActuator( coordinate.getName() )
    model.addForce( controlActuator )
  
  systemState = model.initSystem()

  file = open( 'robot_model.osim', 'w' )
  file.write( model.dump() )
  file.close()

  manager = opensim.Manager( model )
  systemState.setTime( 0 )
  manager.initialize( systemState )
  
  systemState = manager.integrate( 0.001 )

  model.setUseVisualizer( False )
except Exception as e:
  print( e )
