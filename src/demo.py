from dynamic_graph import plug, writeGraph
from dynamic_graph.ros.robot_model import RosRobotModel
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, Task, SOT
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.tools import addTrace
import rospy

def addTrace(device, trace, entityName, signalName, autoRecompute = True):
    """
    Add a signal to a tracer and recompute it automatically if necessary.
    """
    signal = '{0}.{1}'.format(entityName, signalName)
    filename = '{0}-{1}'.format(entityName, signalName)
    trace.add(signal, filename)
    if autoRecompute:
        device.after.addSignal(signal)

I4 =   ((1.,0,0,0),
	(0,1.,0,0),
	(0,0,1.,0),
	(0,0,0,1.),)
	
model = RosRobotModel ('ur5_dynamic')
device = RobotSimu ('ur5_device')

rospy.init_node('fake')
model.loadUrdf ("file:///local/ngiftsun/devel/ros/catkin_ws/src/ur_description/urdf/ur5_robot.urdf")

dimension = model.getDimension ()
device.resize (dimension)

plug (device.state, model.position)
# Set velocity and acceleration to 0
model.velocity.value = dimension * (0.,)
model.acceleration.value = dimension * (0.,)

# Create taks for the base
model.createOpPoint ("base", "waist")

# Create task for the wrist
model.createOpPoint ("wrist", "wrist_3_joint")
feature_wrist = FeaturePosition ('position_wrist', model.wrist, model.Jwrist, I4)
task_wrist = Task ('wrist_task')
task_wrist.controlGain.value = 1.
task_wrist.add (feature_wrist.name)
# Create operational point for the end effector
model.createOpPoint ("ee", "ee_fixed_joint")

solver = SOT ('solver')
solver.setSize (dimension)
solver.push (task_wrist.name)

plug (solver.control, device.control)
device.increment (0.01)

#Create tracer
tracer = TracerRealTime ('trace')
tracer.setBufferSize(2**20)
tracer.open('/tmp/','dg_','.dat')
# Make sure signals are recomputed even if not used in the control graph
device.after.addSignal('{0}.triger'.format(tracer.name))
addTrace (device, tracer, device.name, "state")
addTrace (device, tracer, feature_wrist._feature.name, "position")
addTrace (device, tracer, feature_wrist._reference.name, "position")

#writeGraph('/tmp/graph')

dt = .01
tracer.start ()
for i in range (1000):
	device.increment (dt)
tracer.stop ()
tracer.dump ()

