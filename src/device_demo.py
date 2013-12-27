# -*- coding: utf-8 -*-
"""
Created on Mon Oct 21 11:34:27 2013

@author: bcoudrin
"""
from dynamic_graph import plug
from dynamic_graph.sot.core import RobotSimu
from dynamic_graph.sot.ur.robot import Ur
from dynamic_graph.sot.dyninv import SolverKine
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import TaskJointLimits
#from dynamic_graph.ros.ros_sot_robot_model import Ros

robot = Ur('Ur',device=RobotSimu('ur'))
plug(robot.device.state, robot.dynamic.position)
#ros = Ros(robot)

def toList(sot):
    return map(lambda x: x[1:-1],sot.dispStack().split('|')[1:])
    
SolverKine.toList = toList
sot = SolverKine('sot')
sot.setSize(robot.dimension)

robot.dynamic.velocity.value = robot.dimension*(0.,)
robot.dynamic.acceleration.value = robot.dimension*(0.,)
robot.dynamic.ffposition.unplug()
robot.dynamic.ffvelocity.unplug()
robot.dynamic.ffacceleration.unplug()
robot.dynamic.setProperty('ComputeBackwardDynamics','true')
robot.dynamic.setProperty('ComputeAccelerationCoM','true')
robot.device.control.unplug()
plug(sot.control,robot.device.control)

from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
dt=1e-3
@loopInThread
def inc():
    robot.device.increment(dt)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

# Tasks
taskzero = MetaTaskKine6d('rh',robot.dynamic,'rh','right-wrist')
taskzero.feature.frame('desired')

robot.dynamic.upperJl.recompute(0)
robot.dynamic.lowerJl.recompute(0)
taskJL = TaskJointLimits('taskJL')
plug(robot.dynamic.position,taskJL.position)
taskJL.controlGain.value = 10
taskJL.referenceInf.value = robot.dynamic.lowerJl.value
taskJL.referenceSup.value = robot.dynamic.upperJl.value
taskJL.dt.value = dt
taskJL.selec.value = toFlags(range(18,25)+range(26,27)+range(28,31)+range(32,40)+range(41,42)+range(43,46)+range(47,50))

taskContact = MetaTaskKine6d('contact',robot.dynamic,'contact','left-ankle')
taskContact.feature.frame('desired')
taskContact.feature.selec.value = '011100'
taskContact.gain.setConstant(10)
    
taskFixed = MetaTaskKine6d('contactFixed',robot.dynamic,'contact','left-ankle')
taskFixed.feature.frame('desired')
taskFixed.gain.setConstant(10)

# Problem
from dynamic_graph.sot.core.meta_tasks_kine import gotoNd
targetRH = (0.65,-0.2,0.9)
targetLH = (0.1, 0.0,1.2)
#targetLH = targetRH
gotoNd(taskRH,targetRH,'111',(4.9,0.9,0.01,0.9))
gotoNd(taskLH,targetLH,'111',(4.9,0.9,0.01,0.9))

def push(task):
    if isinstance(task,str): taskName=task
    elif "task" in task.__dict__:  taskName=task.task.name
    else: taskName=task.name
    if taskName not in sot.toList():
        sot.push(taskName)
        if taskName!="taskposture" and "taskposture" in sot.toList():
            sot.down("taskposture")
    return sot

def pop(task):
    if isinstance(task,str): taskName=task
    elif "task" in task.__dict__:  taskName=task.task.name
    else: taskName=task.name
    if taskName in sot.toList(): sot.rm(taskName)    
    return sot

push(taskRH)
push(taskLH)
sot.addContact(taskContact)
push(taskJL)
