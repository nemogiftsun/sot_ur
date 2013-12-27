from dynamic_graph.sot.dynamics.humanoid_robot import AbstractHumanoidRobot
from dynamic_graph import plug, writeGraph
from dynamic_graph.ros.robot_model import RosRobotModel
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, Task, SOT
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.tools import addTrace
import rospy
from dynamic_graph.sot.ur.mobile_robot import Ur



#### demo code ####
robot = Ur('ur5', device=RobotSimu('ur5'))
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
robot_pose = ((1.,0,0,0),
    (0,1.,0,0),
    (0,0,1.,0.089159),
    (0,0,0,1.),)	
feature_waist = FeaturePosition ('position_waist', robot.dynamic.shoulder_pan_joint,robot.dynamic.Jshoulder_pan_joint, robot_pose)
task_waist = Task ('waist_task')
task_waist.controlGain.value = 100.
task_waist.add (feature_waist.name)

# Create task for the lift
I4 =   ((1.,0,0,0.0),
    (0,1.,0,0.136),
    (0,0,1.,0.089),
    (0,0,0,1.),)
feature_lift = FeaturePosition ('position_lift', robot.dynamic.shoulder_lift_joint, robot.dynamic.Jshoulder_lift_joint, I4)
#feature_lift.selec.value = '000000'
task_lift = Task ('lift_task')
task_lift.controlGain.value = 0.
task_lift.add (feature_lift.name)

# Create task for the elbow
I4 =   ((1.,0,0,0.321),
    (0,1.,0,0.109),
    (0,0,1.,0.848),
    (0,0,0,1.),)
feature_elbow = FeaturePosition ('position_elbow', robot.dynamic.elbow_joint, robot.dynamic.Jelbow_joint, I4)
#feature_elbow.selec.value = '000000'
task_elbow = Task ('elbow_task')
task_elbow.controlGain.value = 0.
task_elbow.add (feature_elbow.name)


# Create task for the wrist1
I4 =   ((1.,0,0,0.321),
    (0,1.,0,0.109),
    (0,0,1.,0.848),
    (0,0,0,1.),)
feature_wrist1 = FeaturePosition ('position_wrist1', robot.dynamic.wrist_1_joint, robot.dynamic.Jwrist_1_joint, I4)
#feature_wrist1.selec.value = '000000'
task_wrist1 = Task ('wrist1_task')
task_wrist1.controlGain.value = 0.
task_wrist1.add (feature_wrist1.name)

# Create task for the wrist2
I4 =   ((1.,0,0,0.321),
    (0,1.,0,0.109),
    (0,0,1.,0.848),
    (0,0,0,1.),)
feature_wrist2 = FeaturePosition ('position_wrist2', robot.dynamic.wrist_2_joint, robot.dynamic.Jwrist_2_joint, I4)
#feature_wrist2.selec.value = '000000'
task_wrist2 = Task ('wrist2_task')
task_wrist2.controlGain.value = 0.
task_wrist2.add (feature_wrist2.name)


# Create task for the wrist3
I4 =   ((1.,0,0,0.321),
    (0,1.,0,0.109),
    (0,0,1.,0.848),
    (0,0,0,1.),)
feature_wrist = FeaturePosition ('position_wrist', robot.dynamic.wrist_3_joint, robot.dynamic.Jwrist_3_joint, I4)
task_wrist = Task ('wrist_task')
task_wrist.controlGain.value = 1
task_wrist.add (feature_wrist.name)
#Create tracer
tracer = TracerRealTime ('trace')
tracer.setBufferSize(2**20)
tracer.open('/tmp/','dg_','.dat')
# Make sure signals are recomputed even if not used in the control graph
robot.device.after.addSignal('{0}.triger'.format(tracer.name))
addTrace (robot.device, tracer, robot.device.name, "state")
addTrace (robot.device, tracer, feature_wrist._feature.name, "position")
addTrace (robot.device, tracer, feature_wrist._reference.name, "position")


# solver
solver = SOT ('solver')
solver.setSize (dimension)
solver.push (task_waist.name)
solver.push (task_wrist.name)
solver.push (task_elbow.name)
solver.push (task_lift.name)
solver.push (task_wrist1.name)
solver.push (task_wrist2.name)
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



