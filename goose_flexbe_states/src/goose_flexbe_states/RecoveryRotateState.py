from flexbe_core import EventState, Logger

import rospy
import numpy as np


from geometry_msgs.msg import Twist



class RecoveryRotateState(EventState):
    '''
    State to rotate the robot slightly to detect sheets at another angle.  
    
    <= arrived              required position arrived.
    <= failed               Navigation failed.
    '''

    def __init__(self, rnd_mew=0.0, rnd_std=1.0):
        super(RecoveryRotateState, self).__init__(outcomes=['arrived', 'failed'],
                                            input_keys=['distance', 'x_center'])

        # self._random_angle = rnd_mew + rnd_std* np.random.randn()
        self.movement_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=2)

        self._arrived = False
        self._failed = False

    def on_enter(self, userdata):
        self.counter = 0
        self.direction = np.sign(np.random.randn())
        pass
        

    def execute(self, userdata):
        if self.counter < 10:
            movement = Twist()
            movement.angular.z = self.direction
            self.movement_pub.publish(movement)
            self.counter += 1
        else:
            return 'arrived'
        
        # Otherwise, continue waiting.

    def cancel_active_goals(self):
        pass

    def on_exit(self, userdata):
        self.cancel_active_goals()

    def on_stop(self):
        self.cancel_active_goals()