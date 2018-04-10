#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def taskRequestedCallback(msg):
    global tasksQueue
    global taskRequest
    global readyForTask
    global nextTaskPub
    
    msg = msg.data
    i = msg.find("x")
    j = msg.find("y")
    xPos = int(msg[i+1:j])
    yPos = int(msg[j+1:])
    tasksQueue.append([xPos, yPos])

    print tasksQueue[len(tasksQueue)-1][0]
    print tasksQueue[len(tasksQueue)-1][1]

def readyForTaskCallback(msg):
    global tasksQueue
    global taskRequest
    global readyForTask
    global nextTaskPub

    print len(tasksQueue)
    
    if len(tasksQueue) != 0:
        task = tasksQueue.pop(0)
        data = String()
        data.data = "x" + str(task[0]) + "y" + str(task[1])
        nextTaskPub.publish(data)
    else:
        data = String()
        data.data = "-1"
        nextTaskPub.publish(data)

global tasksQueue
global taskRequest
global readyForTask
global nextTaskPub

tasksQueue = []  

rospy.init_node('taskServer')

taskRequest = rospy.Subscriber('taskRequested', String, taskRequestedCallback)
readyForTask = rospy.Subscriber('readyForTask', String, readyForTaskCallback)
nextTaskPub = rospy.Publisher('nextTask', String, queue_size=1)

rospy.spin()