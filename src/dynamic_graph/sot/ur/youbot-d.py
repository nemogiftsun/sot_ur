from dynamic_graph.sot.dynamics.humanoid_robot import AbstractHumanoidRobot
from dynamic_graph import plug, writeGraph
from dynamic_graph.ros.robot_model import RosRobotModel
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, Task, SOT
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.tools import addTrace
import rospy


class AbstractMobileRobot(object):
    OperationalPoints = []
    AdditionalFrames = []   
    name = None
    initPosition = None
    # initialize robot    
    def initializeRobot(self):
        if not self.dynamic:
            raise RunTimeError("robots models have to be initialized first")

        if not self.device:
            self.device = RobotSimu(self.name + '_device')

        self.device.set(self.initPosition)
        plug(self.device.state, self.dynamic.position)

        self.dynamic.velocity.value = self.dimension*(0.,)
        self.dynamic.acceleration.value = self.dimension*(0.,)

        self.initializeOpPoints(self.dynamic)

    # create operational points
    def initializeOpPoints(self, model):
        for op in self.OperationalPoints:
            model.createOpPoint(op, op)


    # Tracer methods
    def addTrace(self, entityName, signalName):
        if self.tracer:
            self.autoRecomputedSignals.append(
                '{0}.{1}'.format(entityName, signalName))
            addTrace(self, self.tracer, entityName, signalName)
    def startTracer(self):
        """
        Start the tracer if it does not already been stopped.
        """
        if self.tracer:
            self.tracer.start()
    def stopTracer(self):
        """
        Stop and destroy tracer.
        """
        if self.tracer:
            self.tracer.dump()
            self.tracer.stop()
            self.tracer.close()
            self.tracer.clear()
            for s in self.autoRecomputedSignals:
                self.device.after.rmSignal(s)
            self.tracer = None

    # const and deconst
    def __init__(self, name, tracer = None):
        self.name = name
        # Initialize tracer if necessary.
        if tracer:
            self.tracer = tracer

    def __del__(self):
        if self.tracer:
            self.stopTracer()

class youbot(AbstractMobileRobot):
    """
    This class instanciates a Ur5 robot.
    """

    tracedSignals = {
        'dynamic': ["com", "position", "velocity", "acceleration"],
        'device': ['control', 'state']
        }
        
    initPosition = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    def specifySpecialLinks(self):
        if len(self.SpecialLinks) == len(self.SpecialNames):
            for i in range(0,len(self.SpecialLinks)):
                self.dynamic.addJointMapping(self.SpecialLinks[i], self.SpecialNames[i])
        else:
            print 'No Special joints added : SpecialLinks.size != SpecialJoints.size'

    def __init__(self, name, device = None, tracer = None):
        AbstractMobileRobot.__init__ (self, name, tracer)
        # add operational points
        self.OperationalPoints.append('waist')
        self.OperationalPoints.append('arm_joint_5')
        self.OperationalPoints.append('base_joint')

        # device and dynamic model assignment
        self.device = device
        self.dynamic = RosRobotModel("{0}_dynamic".format(name))
        # load model
        self.dynamic.loadFromParameterServer()
        self.dimension = self.dynamic.getDimension()
        self.initPosition = (0.,) * self.dimension
        # initialize ur robot
        self.initializeRobot()
__all__ = ["Ur"]


#### demo code ####
# 1. Instanciate a Pr2
# The URDF description of the robot must have 
# been loaded in robot_description parameter
# on the Ros Parameter Server
from dynamic_graph.sot.pr2.robot import Pr2
from dynamic_graph.sot.core import RobotSimu
from dynamic_graph import plug
robot = youbot('youbot', device=RobotSimu('youbot'))
plug(robot.device.state, robot.dynamic.position)

# 2. Ros binding
# roscore must be running
from dynamic_graph.ros import Ros
ros = Ros(robot)

# 3. Create a solver
from dynamic_graph.sot.application.velocity.precomputed_tasks import Solver
solver = Solver(robot)

# 4. Define a position task for the right hand
from dynamic_graph.sot.core.meta_tasks_kine import gotoNd, MetaTaskKine6d
from numpy import eye
from dynamic_graph.sot.core.matrix_util import matrixToTuple

taskRH=MetaTaskKine6d('rh',robot.dynamic,'rh','arm_joint_5')
Pr2handMgrip = eye(4); Pr2handMgrip[0:3,3] = (0.337,0,0.275)
taskRH.opmodif = matrixToTuple(Pr2handMgrip)
taskRH.feature.frame('desired')
targetR=(0.65,0.2,0.9)
selec='111'
gain=(4.9,0.9,0.01,0.9)
gotoNd(taskRH,targetR,selec,gain)

# 5. Add a contact constraint with the robot and the floor
contact = MetaTaskKine6d('contact',robot.dynamic,'contact','left-ankle')
contact.feature.frame('desired')
contact.feature.selec.value = '011100'
contact.gain.setConstant(10)
contact.keep()
locals()['contactBase'] = contact

# 6. Push tasks in the solver
solver.push(taskRH.task)
solver.push(contactBase.task)

# Main loop
dt=3e-3
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread
def inc():
    robot.device.increment(dt)
    
runner=inc()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)

print 'Type go to run the solver loop'



