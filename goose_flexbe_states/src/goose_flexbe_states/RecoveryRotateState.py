from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped


class RecoveryRotateState(EventState):
    '''
    State to rotate the robot slightly to detect sheets at another angle.  
    
    <= arrived              required position arrived.
    <= failed               Navigation failed.
    '''

    def __init__(self, rnd_mew=0.0, rnd_std=1.0):
        super(RecoveryRotateState, self).__init__(outcomes=['arrived', 'failed'],
                                            input_keys=['distance', 'x_center'])
        # Set up communication with move_base
        self._action_topic = "/move_base"

        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})

        self._random_angle = rnd_mew + rnd_std* np.random.randn()

        self._arrived = False
        self._failed = False

    def calculate_relative_movement_goal(self):
        relative_goal = PoseStamped()
        relative_goal.header.frame_id = "base_link"  

        q = quaternion_from_euler(0, 0, self._random_angle)
        relative_goal.pose.orientation.x = q[0]
        relative_goal.pose.orientation.y = q[1]
        relative_goal.pose.orientation.z = q[2]
        relative_goal.pose.orientation.w = q[3]
        return relative_goal
    
    def relative_2_map_goal(self, relative_goal):
        # Transform goal from base_link to map
        try:
            tf_buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tf_buffer)
            
            # Wait for the transform (timeout 1s)
            rospy.sleep(1.0)  
            transform = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            
            transformed_goal = tf2_geometry_msgs.do_transform_pose(relative_goal, transform)
            # transformed_goal.header.stamp = rospy.Time.now()
            return transformed_goal
        
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
            self._failed = True
        except tf2_ros.ConnectivityException as e:
            rospy.logerr(f"Transform connectivity issue: {e}")
            self._failed = True
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr(f"Transform extrapolation error: {e}")
            self._failed = True


    def on_enter(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # Make sure this matches SLAM map frame.
        goal.target_pose.header.stamp = rospy.Time.now()

        relative_goal = self.calculate_relative_movement_goal()
        goal.target_pose = self.relative_2_map_goal(relative_goal)
        
        # Send the action goal for execution
        try:
            Logger.loginfo("Sending goal to sheet")
            self._client.send_goal(self._action_topic, goal)
        except Exception as e:
            Logger.logwarn("Unable to send navigation action goal:\n%s" % str(e))
            self._failed = True

    def execute(self, userdata):
        if self._arrived:
            return 'arrived'
        if self._failed:
            return 'failed'

        if self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                self._arrived = True
                return 'arrived'
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn('Navigation failed: %s' % str(status))
                self._failed = True
                return 'failed'
        # Otherwise, continue waiting.

    def cancel_active_goals(self):
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo('Cancelled move_base active action goal.')

    def on_exit(self, userdata):
        self.cancel_active_goals()

    def on_stop(self):
        self.cancel_active_goals()