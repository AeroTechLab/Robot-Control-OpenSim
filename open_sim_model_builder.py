#!/usr/bin/env python2 

import sys
import os

import simulation as sim
import tools
import analyses
import simbody as simtk

print( sim )
print( simtk )

m = sim.Model( 'data/arm26.osim' )
# Silence warning messages if mesh (.vtp) files cannot be found.
# sim.Model.setDebugLevel( 0 )

#m.printBasicInfo( sys.stdout )

muscle = sim.Thelen2003Muscle()
print( muscle.getPennationModel().get_optimal_fiber_length() )
print( muscle.getActivationModel().get_activation_time_constant() )

cmc = tools.CMCTool()
print( cmc )
print( m.getName() )
cmc.setModel( m )
reporter = analyses.ForceReporter()
reporter.setName( 'force' )
cmc.getAnalysisSet().adoptAndAppend( reporter )
print( cmc.getAnalysisSet().get( 0 ).getName() )

#class TestBasics(unittest.TestCase):

    #def test_AnalysisToolModel(self):
        ## Test analyses module.
        #cmc = osim.CMCTool()
        #model = osim.Model()
        #model.setName('eggplant')
        #fr = osim.ForceReporter()
        #fr.setName('strong')
        #cmc.setModel(model)
        #cmc.getAnalysisSet().adoptAndAppend(fr)

        #assert cmc.getModel().getName() == 'eggplant'
        #assert cmc.getAnalysisSet().get(0).getName() == 'strong'

    #def test_ManagerConstructorCreatesIntegrator(self):
        ## Make sure that the Manager is able to create a default integrator.
        ## This tests a bug fix: previously, it was impossible to use the
        ## Manager to integrate from MATLAB/Python, since it was not possible
        ## to provide an Integrator to the Manager.
        #model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        #state = model.initSystem()

        #manager = osim.Manager(model)
        #manager.setInitialTime(0)
        #manager.setFinalTime(0.00001)
        #manager.integrate(state)

    #def test_WrapObject(self):
        ## Make sure the WrapObjects are accessible.
        #model = osim.Model()

        #sphere = osim.WrapSphere()
        #model.getGround().addWrapObject(sphere)

        #cylinder = osim.WrapCylinder()
        #cylinder.set_radius(0.5)
        #model.getGround().addWrapObject(cylinder)

        #torus = osim.WrapTorus()
        #model.getGround().addWrapObject(torus)

        #ellipsoid = osim.WrapEllipsoid()
        #model.getGround().addWrapObject(ellipsoid)
