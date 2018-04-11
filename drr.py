#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import smach
import smach_ros
import cv2
from time import sleep
import os
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

def move(x,y):
    global client
    pose = [(x,y,0.0),(0.0,0.0,0.0,1.0)]
    print(pose)
    goal = goal_pose(pose)
    client.send_goal(goal)
    client.wait_for_result()
    return

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose


#define State WaitForRequest
class WaitForRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['returnToBase', 'moveToRequest'],
                                    output_keys=['taskOut'])
        self.readyForTaskPub = rospy.Publisher('readyForTask', String, queue_size=1)
        self.nextTaskSub = rospy.Subscriber('nextTask', String, self.nextTaskCallback)
        self.counter = 0
        self.task = ""

    def nextTaskCallback(self, msg):
        self.task = msg.data
        print self.task

    def sendRequest(self):
        msg = String()
        msg.data = "ready"
        self.readyForTaskPub.publish(msg)


    def execute(self, userdata):
        global docked
        rospy.loginfo('Executing state WAIT_FOR_REQUEST')
        self.task = "-1"
        self.counter = 0
        
        while((self.task == "-1" and self.counter <= 10) or (self.task == "-1" and docked)):
            self.sendRequest()
            sleep(1)
            self.counter += 1
            print "waiting"
        
        if(self.task == "-1"):
            return 'returnToBase'
        else:
            docked = False
            print self.task
            i = self.task.find("x")
            j = self.task.find("y")
            xPos = int(self.task[i+1:j])
            yPos = int(self.task[j+1:])
            userdata.taskOut = [float(xPos)*0.05 - 25, float(yPos)*-0.05 + 23]
            return 'moveToRequest'

#define State ReturnToBase
class ReturnToBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dockAtBase'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state RETURN_TO_BASE')
        
        #move to "docking location"
        sleep(2)
        return 'dockAtBase'

#define State DockAtBase
class DockAtBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waitForRequest'])
        
    def execute(self, userdata):
        global docked
        rospy.loginfo('Executing state DOCK_AT_BASE')
        #run os system thing
        sleep(2)
        docked = True
        return 'waitForRequest'

#define State MoveToRequest
class MoveToRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waitForRequest', 'waitForLoad'],
                                    input_keys=['taskIn'],
                                    output_keys=['taskStart'])
        self.counter = 0
        self.task = ""

    def execute(self, userdata):
        #Remove when done testing
        #return 'waitForLoad'
        #//
        rospy.loginfo('Executing state MOVE_TO_REQUEST')
        self.task = userdata.taskIn
        
        #adding the move here
        move(self.task[0],self.task[1])
        print self.task
        userdata.taskStart = self.task
        return 'waitForLoad'
        
        '''
        if self.counter == 0:
            self.counter += 1
            sleep(2)
            return 'waitForRequest'
        elif self.counter == 1:
            self.counter = 0
            sleep(2)
            userdata.taskStart = self.task
            return 'waitForLoad'
        '''
#define State WaitForLoad
class WaitForLoad(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waitForRequest', 'makeDelivery'],
                                    input_keys=['taskStart'],
                                    output_keys=['destOut', 'taskStart'])
        self.counter = 0
        self.xPos = -1
        self.yPos = -1
        self.haveCursor = True

    def getCursor(self, event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.xPos = x
            self.yPos = y
            self.haveCursor = False
            print x
            print y
        
    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT_FOR_LOAD')
        self.xPos = -1
        self.yPos = -1
        self.haveCursor = True
        os.system("clear")
        cancel = raw_input('Press enter when loaded (q for cancellation): ')

        if(cancel != 'q'):
            cv2.namedWindow("Map")
            cv2.setMouseCallback("Map", self.getCursor)
            map = cv2.imread("map.png")
            cv2.imshow("Map", map)

            while(self.haveCursor):   
                k = cv2.waitKey(5) & 0xFF
                if k == 27:
                    break

            cv2.destroyWindow("Map")

            if(self.xPos != -1):
                userdata.destOut = [float(self.xPos)*0.05 - 25, float(self.yPos)*-0.05 + 23]
                return 'makeDelivery'

        return 'waitForRequest'

#define State MakeDelivery
class MakeDelivery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['returnToSender', 'waitForUnload'],
                                    input_keys=['destIn', 'taskStart'],
                                    output_keys=['taskStart'])
        self.counter = 0
        self.dest = []

    def execute(self, userdata):
        rospy.loginfo('Executing state MAKE_DELIVERY')
        self.dest = userdata.destIn
        move(self.dest[0],self.dest[1])
        return 'waitForUnload'
        
        '''
        if self.counter == 0:
            self.counter += 1
            sleep(2)
            return 'waitForUnload'
        elif self.counter == 1:
            self.counter = 0
            sleep(2)
            return 'waitForUnload'
        '''
#define ReturnToSender
class ReturnToSender(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waitForLoad', 'waitForRequest'],
                                    input_keys=['taskStart'],
                                    output_keys=['taskStart'])
        self.counter = 0
        self.start = [0,0]
    def execute(self, userdata):
        rospy.loginfo('Executing state RETURN_TO_SENDER')
        self.start = userdata.taskStart
        move(self.start[0],self.start[1])
        return 'waitForRequest'
        '''
        if self.counter == 0:
            self.counter += 1
            sleep(2)
            return 'waitForLoad'
        elif self.counter == 1:
            self.counter = 0
            sleep(2)
            return 'waitForRequest'
        '''
#define WaitForUnload
class WaitForUnload(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['returnToSender', 'waitForRequest'],
                                    input_keys=['taskStart'],
                                    output_keys=['taskStart'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT_FOR_UNLOAD')
        os.system("clear")
        response = raw_input('Press enter when unloaded (q to Return to Sender): ')

        if(response != 'q'):
            return 'waitForRequest'

        return 'returnToSender'

def main():
    global docked
    global client
    docked = True
    rospy.init_node('jeeves_botSM')
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    init_posepub = rospy.Publisher('initialpose',PoseWithCovarianceStamped,queue_size =1)
    initpose = Pose()
    initpose.position.x = -1.80218
    initpose.position.y = -11.0176
    initpose.position.z = 0
    initpose.orientation.x = 0
    initpose.orientation.y = 0
    initpose.orientation.z = -0.5435744
    initpose.orientation.w = 0.839361
    inPose = PoseWithCovarianceStamped()
    inPose.pose.pose = initpose
    #rospy.wait_for_service('clear_costmaps')
    #clear_costmaps = rospy.ServiceProxy('clear_costmaps',Empty)
    #clear_costmaps()
    init_posepub.publish(inPose)
    
    #Create a smach state machine
    sm_stk = smach.StateMachine(outcomes=['exit'])
    with sm_stk:
        #Add states to the container
        smach.StateMachine.add('WAIT_FOR_REQUEST', WaitForRequest(), 
                                transitions={'returnToBase':'RETURN_TO_BASE','moveToRequest':'MOVE_TO_REQUEST'},
                                remapping={'taskOut':'task'})
        smach.StateMachine.add('RETURN_TO_BASE', ReturnToBase(), 
                                transitions={'dockAtBase':'DOCK_AT_BASE'})
        smach.StateMachine.add('DOCK_AT_BASE', DockAtBase(), 
                                transitions={'waitForRequest':'WAIT_FOR_REQUEST'})
        smach.StateMachine.add('MOVE_TO_REQUEST', MoveToRequest(), 
                                transitions={'waitForRequest':'WAIT_FOR_REQUEST','waitForLoad':'WAIT_FOR_LOAD'},
                                remapping={'taskIn':'task','taskStart':'taskStart'})
        smach.StateMachine.add('WAIT_FOR_LOAD', WaitForLoad(), 
                                transitions={'waitForRequest':'WAIT_FOR_REQUEST','makeDelivery':'MAKE_DELIVERY'},
                                remapping={'destOut':'dest','taskStart':'taskStart'})
        smach.StateMachine.add('MAKE_DELIVERY', MakeDelivery(), 
                                transitions={'returnToSender':'RETURN_TO_SENDER','waitForUnload':'WAIT_FOR_UNLOAD'},
                                remapping={'destIn':'dest','taskStart':'taskStart'})
        smach.StateMachine.add('RETURN_TO_SENDER', ReturnToSender(), 
                                transitions={'waitForLoad':'WAIT_FOR_LOAD','waitForRequest':'WAIT_FOR_REQUEST'},
                                remapping={'taskStart':'taskStart'})
        smach.StateMachine.add('WAIT_FOR_UNLOAD', WaitForUnload(), 
                                transitions={'returnToSender':'RETURN_TO_SENDER','waitForRequest':'WAIT_FOR_REQUEST'},
                                remapping={'taskStart':'taskStart'})

    sis = smach_ros.IntrospectionServer('JEEVES_server', sm_stk, 'drr')
    sis.start()

    outcome = sm_stk.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
