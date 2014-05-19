from dynamic_graph.sot.youbot.robot import youbot
from dynamic_graph.sot.pr2.robot import Pr2
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
#robot = Pr2('youbot', device=RobotSimu('youbot'))
dimension = robot.dynamic.getDimension()
robot.device.resize (dimension)
from dynamic_graph.ros import Ros
ros = Ros(robot)
#waisttask-1

robot_pose = ((1.,0,0,0.4),
    (0,1.,0,0),
    (0,0,1.,0.0),
    (0,0,0,1.),)	
feature_waist = FeaturePosition ('position_waist', robot.dynamic.base_joint,robot.dynamic.Jbase_joint, robot_pose)
task_waist = Task ('waist_task')
task_waist.controlGain.value = 0.5
task_waist.add (feature_waist.name)

#waisttask
task_waist_metakine=MetaTaskKine6d('task_waist_metakine',robot.dynamic,'base_joint','base_joint')
goal_waist = ((1.,0,0,0.0),(0,1.,0,-0.0),(0,0,1.,0.0),(0,0,0,1.),)
task_waist_metakine.feature.frame('desired')
#task_waist_metakine.feature.selec.value = '011101'#RzRyRxTzTyTx
task_waist_metakine.gain.setConstant(0.5)
task_waist_metakine.featureDes.position.value = goal_waist
solver = SolverKine('sot_solver')
solver.setSize (robot.dynamic.getDimension())
robot.device.resize (robot.dynamic.getDimension())
'''
#taskinequality
task_waist=TaskInequality('taskcollision')
collision_feature = FeatureGeneric('collisionfeature')
plug(a.collisionJacobian,collision_feature.jacobianIN)
plug(a.collisionDistance,collision_feature.errorIN)
task_collision_avoidance.add(collision_feature.name)
task_collision_avoidance.referenceInf.value = (0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05)    
task_collision_avoidance.referenceSup.value = (2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10)  
task_collision_avoidance.dt.value=1
task_collision_avoidance.controlGain.value=50.0
'''
# ll
ll = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0100692,0.0100692,-5.02655,0.0221239,0.110619,0,0)
hl = (2e10,2e10,2e10,2e10,2e10,2e10,5.84014,2.61799,-0.015708,3.4292,5.64159,0,0)
taskjl = TaskJointLimits('taskJL')
plug(robot.dynamic.position,taskjl.position)
taskjl.controlGain.value = 10
taskjl.referenceInf.value = ll
taskjl.referenceSup.value = hl
taskjl.dt.value = 1 
#armtask
task_wrist_metakine=MetaTaskKine6d('task_wrist_metakine',robot.dynamic,'arm_joint_5','arm_joint_5')
ip = (0.200, -0.006, 0.377,-0.155, 0.414, -0.555)
zp = (0.198, 0.001, 0.581 ,-0.000, 0.001, 0.010)
prl = (0.606, -0.001, 0.425,-3.115, 0.228, -3.029)
aov = (-0.136, -0.005, 0.389,0.005, -0.575, 0.010)
pp = (0.567, -0.013, 0.260,0,0,0)
pr = (0.008,0.04,0.515,0,0,0)
goal = matrixToTuple(generic6dReference(zp))
print goal
#solver.damping.value =3e-2
task_wrist_metakine.gain.setConstant(1)
task_wrist_metakine.feature.selec.value = '000111'
task_wrist_metakine.featureDes.position.value = goal
#solver

#solver.push (task_waist.name)
solver.push (taskjl.name)
solver.push (task_waist_metakine.task.name)
solver.push (task_wrist_metakine.task.name)
plug (solver.control,robot.device.control)
dt = 0.01

from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread

def inc():
    robot.device.increment(dt)

runner=inc()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)


