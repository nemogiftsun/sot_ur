#from dynamic_graph.sot.dynamics.humanoid_robot import AbstractHumanoidRobot
from dynamic_graph import plug, writeGraph
from dynamic_graph.ros.robot_model import RosRobotModel
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, Task, SOT
#import dynamic_graph.sot.core.FeaturePoint6dRelative
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
robot = youbot('youbot', device=RobotSimu('youbot'))
dimension = robot.dynamic.getDimension()
robot.device.resize (dimension)

# 2. Ros binding
# roscore must be running
from dynamic_graph.ros import Ros
ros = Ros(robot)





def addTrace(device, trace, entityName, signalName, autoRecompute = True):
    """
    Add a signal to a tracer and recompute it automatically if necessary.
    """
    signal = '{0}.{1}'.format(entityName, signalName)
    filename = '{0}-{1}'.format(entityName, signalName)
    trace.add(signal, filename)
    if autoRecompute:
        device.after.addSignal(signal)

# Create task for the waist


robot_pose = ((1.,0,0,500),(0,1.,0,0),(0,0,1.,0),(0,0,0,1.),)	
feature_waist = FeaturePosition ('position_waist', robot.dynamic.base_joint, robot.dynamic.Jbase_joint, robot_pose)
feature_waist.selec.value = '001110'
task_waist = Task ('waist_task')
task_waist.controlGain.value = 5
task_waist.add (feature_waist.name)
'''
feature_waist = MetaTaskKine6d('contact',robot.dynamic,'contact','base_joint')
feature_waist.feature.frame('desired')
feature_waist.feature.selec.value = '011100'
feature_waist.gain.setConstant(10)
    #locals()['contact'] = task
    return task
'''

# Create task for the wrist
'''					
I4 =   ((1.,0,0,0.637),(0,1.,0,0.0),(0,0,1.,0.275),(0,0,0,1.),)
'''	
I4 =   ((1.,0,0,030),
	(0,1.,0,0.),
	(0,0,1.,0.600),
	(0,0,0,1.),)
	
feature_wrist = FeaturePosition ('position_wrist', robot.dynamic.arm_joint_5, robot.dynamic.Jarm_joint_5, I4)
task_wrist = Task ('wrist_task')
task_wrist.controlGain.value = 1
task_wrist.add (feature_wrist.name)

'''
#Create tracer
tracer = TracerRealTime ('trace')
tracer.setBufferSize(2**20)
tracer.open('/tmp/','dg_','.dat')
# Make sure signals are recomputed even if not used in the control graph
robot.device.after.addSignal('{0}.triger'.format(tracer.name))
addTrace (robot.device, tracer, robot.device.name, "state")
addTrace (robot.device, tracer, feature_wrist._feature.name, "position")
addTrace (robot.device, tracer, feature_wrist._reference.name, "position")

example code

from dynamic_graph.sot.youbot.robot import youbot
from dynamic_graph.ros.robot_model import RosRobotModel
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, Task, SOT
from dynamic_graph.sot.core.meta_tasks import generic6dReference
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import SolverKine
robot = youbot('youbot', device=RobotSimu('youbot'))
dimension = robot.dynamic.getDimension()
robot.device.resize (dimension)
from dynamic_graph.ros import Ros
ros = Ros(robot)
task_waist_metakine=MetaTaskKine6d('task_waist_metakine',robot.dynamic,'base_joint','base_joint')
goal_waist = ((1.,0,0,-0.2),(0,1.,0,-0.0),(0,0,1.,0),(0,0,0,1.),)
#task_waist_metakine.feature.selec.value = '111111'#RzRyRxTzTyTx
task_waist_metakine.gain.setConstant(1)
task_waist_metakine.featureDes.position.value = goal_waist
solver = SolverKine('sot_solver')
solver.setSize (robot.dynamic.getDimension())
robot.device.resize (robot.dynamic.getDimension())
solver.push (task_waist_metakine.task.name)
plug (solver.control,robot.device.control)

'''




# solver
solver = SOT ('solver')
solver.setSize (dimension)
solver.push (task_waist.name)
solver.push (task_wrist.name)
plug (solver.control, robot.device.control)
#robot.device.increment (0.01)

dt = 0.01
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread

def inc():
    robot.device.increment(dt)

runner=inc()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)





