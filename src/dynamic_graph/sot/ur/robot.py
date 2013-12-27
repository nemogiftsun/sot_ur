from dynamic_graph.sot.dynamics.humanoid_robot import AbstractHumanoidRobot
from dynamic_graph.ros.robot_model import RosRobotModel
#to be removed later
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, Task, SOT
from dynamic_graph import plug, writeGraph

class Ur(AbstractHumanoidRobot):
    """
    This class instanciates a Ur5 robot.
    """
    OperationalPoints = ['waist','wrist_3_joint']
    
    tracedSignals = {
        'dynamic': ["com", "position", "velocity", "acceleration"],
        'device': ['control', 'state']
        }
        
    def initializeUrRobot(self):
        """
        initialize ur robot
        """
        if not self.dynamic:
            raise RunTimeError("robots models have to be initialized first")
        if not self.device:
            self.device = RobotSimu(self.name + '_device')
        plug(self.device.state, self.dynamic.position)
        self.dynamic.velocity.value = self.dimension*(0.,)
        self.dynamic.acceleration.value = self.dimension*(0.,)
        self.initializeOpPoints(self.dynamic)

    def __init__(self, name, device = None, tracer = None):
        AbstractHumanoidRobot.__init__ (self, name, tracer)
        self.device = device
        self.dynamic = RosRobotModel("{0}_dynamic".format(name))
        #self.specifySpecialLinks()
        self.dynamic.loadFromParameterServer()
        self.dimension = self.dynamic.getDimension()
        self.initializeUrRobot()
__all__ = ["Ur"]


##########################
####### demo code ########
##########################
robot = Ur('Ur5', device=RobotSimu('Ur5'))

