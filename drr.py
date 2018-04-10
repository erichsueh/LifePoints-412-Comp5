#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import smach
import smach_ros
from time import sleep

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
        
        rospy.loginfo('Executing state WAIT_FOR_REQUEST')
        self.task = "-1"
        self.counter = 0
        msg = String()
        msg.data = "ready"
        print "sending ready"
        print msg.data
        while(self.task == "-1" and self.counter <= 10):
            self.sendRequest()
            sleep(1)
            self.counter += 1
            print "waiting"
        
        if(self.task == "-1"):
            return 'returnToBase'
        else:
            print self.task
            userdata.taskOut = self.task
            return 'moveToRequest'

#define State ReturnToBase
class ReturnToBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dockAtBase', 'moveToRequest'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state RETURN_TO_BASE')

        if self.counter == 0:
            self.counter += 1
            sleep(2)
            return 'dockAtBase'
        elif self.counter == 1:
            self.counter = 0
            sleep(2)
            return 'moveToRequest'

#define State DockAtBase
class DockAtBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waitForRequest'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state DOCK_AT_BASE')

        sleep(2)
        return 'waitForRequest'

#define State MoveToRequest
class MoveToRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waitForRequest', 'waitForLoad'],
                                    input_keys=['taskIn'])
        self.counter = 0
        self.task = ""

    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE_TO_REQUEST')
        self.task = userdata.taskIn
        print self.task
        
        if self.counter == 0:
            self.counter += 1
            sleep(2)
            return 'waitForRequest'
        elif self.counter == 1:
            self.counter = 0
            sleep(2)
            return 'waitForLoad'

#define State WaitForLoad
class WaitForLoad(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waitForRequest', 'makeDelivery'])
        self.counter = 0
        
    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT_FOR_LOAD')
        
        if self.counter == 0:
            self.counter += 1
            sleep(2)
            return 'waitForRequest'
        elif self.counter == 1:
            self.counter = 0
            sleep(2)
            return 'makeDelivery'

#define State MakeDelivery
class MakeDelivery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['returnToSender', 'waitForUnload'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state MAKE_DELIVERY')

        if self.counter == 0:
            self.counter += 1
            sleep(2)
            return 'returnToSender'
        elif self.counter == 1:
            self.counter = 0
            sleep(2)
            return 'waitForUnload'

#define ReturnToSender
class ReturnToSender(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waitForLoad', 'waitForRequest'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state RETURN_TO_SENDER')

        if self.counter == 0:
            self.counter += 1
            sleep(2)
            return 'waitForLoad'
        elif self.counter == 1:
            self.counter = 0
            sleep(2)
            return 'waitForRequest'

#define WaitForUnload
class WaitForUnload(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['returnToSender', 'waitForRequest'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT_FOR_UNLOAD')

        if self.counter == 0:
            self.counter += 1
            sleep(2)
            return 'returnToSender'
        elif self.counter == 1:
            self.counter = 0
            sleep(2)
            return 'waitForRequest'

def main():
    rospy.init_node('jeeves_botSM')

    #Create a smach state machine
    sm_stk = smach.StateMachine(outcomes=['exit'])
    with sm_stk:
        #Add states to the container
        smach.StateMachine.add('WAIT_FOR_REQUEST', WaitForRequest(), 
                                transitions={'returnToBase':'RETURN_TO_BASE','moveToRequest':'MOVE_TO_REQUEST'},
                                remapping={'taskOut':'task'})
        smach.StateMachine.add('RETURN_TO_BASE', ReturnToBase(), 
                                transitions={'dockAtBase':'DOCK_AT_BASE','moveToRequest':'MOVE_TO_REQUEST'})
        smach.StateMachine.add('DOCK_AT_BASE', DockAtBase(), 
                                transitions={'waitForRequest':'WAIT_FOR_REQUEST'})
        smach.StateMachine.add('MOVE_TO_REQUEST', MoveToRequest(), 
                                transitions={'waitForRequest':'WAIT_FOR_REQUEST','waitForLoad':'WAIT_FOR_LOAD'},
                                remapping={'taskIn':'task'})
        smach.StateMachine.add('WAIT_FOR_LOAD', WaitForLoad(), 
                                transitions={'waitForRequest':'WAIT_FOR_REQUEST','makeDelivery':'MAKE_DELIVERY'})
        smach.StateMachine.add('MAKE_DELIVERY', MakeDelivery(), 
                                transitions={'returnToSender':'RETURN_TO_SENDER','waitForUnload':'WAIT_FOR_UNLOAD'})
        smach.StateMachine.add('RETURN_TO_SENDER', ReturnToSender(), 
                                transitions={'waitForLoad':'WAIT_FOR_LOAD','waitForRequest':'WAIT_FOR_REQUEST'})
        smach.StateMachine.add('WAIT_FOR_UNLOAD', WaitForUnload(), 
                                transitions={'returnToSender':'RETURN_TO_SENDER','waitForRequest':'WAIT_FOR_REQUEST'})

    sis = smach_ros.IntrospectionServer('JEEVES_server', sm_stk, 'drr')
    sis.start()

    outcome = sm_stk.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()