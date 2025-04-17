from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
import rospy
import math
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped


class MoveToSheet(EventState):
    '''
    State to send a goal to the move_base action server to move the robot to a home position.
    Home position is hard coded and known.

    -- camera_width     int     the width of the camera in pixels
    -- camera_horiz_FOV float   the field of view of the camera horzontally

    ># distance     float   distance from the detected sheet
    ># x_center     float   the center of the bounding box     
    
    <= arrived              required position arrived.
    <= failed               Navigation failed.
    '''

    def __init__(self, camera_width, camera_horiz_FOV, distance_scaler):
        super(MoveToSheet, self).__init__(outcomes=['arrived', 'failed'],
                                            input_keys=['distance', 'x_center'])
        # Set up communication with move_base
        self._action_topic = "/move_base"

        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})

        self.width = camera_width
        self.horizontal_FOV = camera_horiz_FOV
        self.distance_scaler = distance_scaler

        self._arrived = False
        self._failed = False

    def calculate_relative_movement_goal(self, userdata):
        Logger.loginfo(f"Recieved sheet: Distance: {userdata.distance}, center: {userdata.x_center}")
        relative_goal = PoseStamped()
        relative_goal.header.frame_id = "base_link"  
        # relative_goal.header.stamp = rospy.Time.now()

        angle = (userdata.x_center - self.width / 2) * self.horizontal_FOV / self.width
        rospy.loginfo(f"Angle: {angle} degrees")
        relative_goal.pose.position.x = self.distance_scaler * math.cos(angle * math.pi / 180) * (userdata.distance / 1000)
        relative_goal.pose.position.y = self.distance_scaler * -math.sin(angle * math.pi / 180) * (userdata.distance / 1000)
        rospy.loginfo(f"Goal position: {relative_goal.pose.position.x}m, {relative_goal.pose.position.y}m")
        relative_goal.pose.orientation.w = 1.0
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

        relative_goal = self.calculate_relative_movement_goal(userdata)
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