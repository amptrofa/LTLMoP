#!/usr/bin/env python

import sys,os

from direct.showbase.ShowBase import ShowBase
from panda3d.ode import *
from panda3d.core import *
from direct.task import Task


# needs to add the path of ltlmop_root to sys path
sys.path.append('../../../..')
import lib.regions

info = """SiMORES

SMORES Simulator for LTLMoP
[Adam Trofa - Cornell University]
[Built from CKBot Simulator]
[Autonomous Systems Laboratory - 2013]

"""

class SiMORES(ShowBase):
    """
    SMORES Simulator Class
    
    Units are consistent in (g/cm/s)
    """
    
    def __init__(self,debug=False):
        ShowBase.__init__(self)
        
        if debug:
            self.setFrameRateMeter(True)
                
        self.setBackgroundColor(1,0,.8)
        self.sunlight = DirectionalLight("SunLight")
        self.sunlight.setDirection(Vec3(-1,-1,-1))
        self.sunlightNP = self.render.attachNewNode(self.sunlight)
        self.ambientLight = AmbientLight("AmbientLight")
        self.ambientLight.setColor(VBase4(.8,.8,.8,1))
        self.ambientLightNP = self.render.attachNewNode(self.ambientLight)
        # Enable lights for entire tree
        self.render.setLight(self.ambientLightNP)
        self.render.setLight(self.sunlightNP)

        # Set up the world
        self.world = OdeWorld()
        self.world.setGravity(0,0,-98.0)
        self.world.initSurfaceTable(1)
        self.world.setSurfaceEntry(0,0,1,.2,150,0,0,0,.002)
        self.space = OdeSimpleSpace() # for large number of bodies, consider OdeQuatTreeSpace()
        self.space.setAutoCollideWorld(self.world) # Collisions occur in this world (seems redundant)
        self.contactGroup = OdeJointGroup()
        self.space.setAutoCollideJointGroup(self.contactGroup)

        # Create a ground plane
        self.groundGeom = OdePlaneGeom(self.space, 0, 0, 1, 0) # space, normal x, n y, n z, offset
        self.groundCard = CardMaker("ground")
        self.groundCard.setColor(1,0,0,.5)
        self.groundCard.setFrame(Point3(100,100,0),Point3(-100,100,0),Point3(-100,-100,0),Point3(100,-100,0))
        self.ground = self.render.attachNewNode(self.groundCard.generate())
        
        # Add our test model
        self.testModel = self.loader.loadModel(Filename.fromOsSpecific(os.path.abspath("")) + "/models/case")
        self.testModel.setPos(self.render,0,0,100)
        self.testModel.reparentTo(self.render)
        self.testModelBody = OdeBody(self.world)
        modelMass = OdeMass()
        modelMass.setBox(10,5,5,5)
        self.testModelBody.setMass(modelMass)
        self.testModelBody.setPosition(self.testModel.getPos())
        testModelGeom = OdeTriMeshGeom(self.space,OdeTriMeshData(self.testModel))
        testModelGeom.setBody(self.testModelBody)

        self.taskMgr.add(self.simPhysics,"SimulationPhysics")
        
        self.disableMouse()
        self.camera.setPos(200,200,200)
        self.camera.lookAt(0,0,0)
        mat = Mat4(self.camera.getMat())
        mat.invertInPlace()
        self.mouseInterfaceNode.setMat(mat)
        self.enableMouse()


    def simPhysics(self,task):
        self.space.autoCollide()
        self.world.quickStep(globalClock.getDt())
        
        self.testModel.setPosQuat(self.render, self.testModelBody.getPosition(), Quat(self.testModelBody.getQuaternion()))

        self.contactGroup.empty()

        return Task.cont

    def runSim(self):
        self.run()

if (__name__ == '__main__'):
    """
    Instantiates a simulator and runs it in stand-alone mode with all default arguments.
    """
    
    debug = False
    if len(sys.argv) > 1 and sys.argv[1] == "debug":
        debug = True

    sim = SiMORES(debug)
    sim.runSim()
