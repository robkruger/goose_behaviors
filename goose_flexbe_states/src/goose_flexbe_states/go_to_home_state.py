from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
import rospy


class GoToHomeState(EventState):
    '''
    State to send a goal to the move_base action server to move the robot to a home position.
    Home position is hard coded and known.

    <= arrived            Home position arrived.
    <= failed             Navigation failed.
    '''

    def __init__(self):
        super(GoToHomeState, self).__init__(outcomes=['arrived', 'failed'])
        # Here, you would set up the SimpleActionState to communicate with move_base
        self._action_topic = "/move_base"

        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})

        self._arrived = False
        self._failed = False

    def on_enter(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # Make sure this matches SLAM map frame.
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = -1.030534267425537
        goal.target_pose.pose.position.y = -1.6209702491760254
        
        goal.target_pose.pose.orientation.z = 0.9673755124939971
        goal.target_pose.pose.orientation.w = 0.2533468330707462

        # Send the action goal for execution
        try:
            Logger.loginfo("Sending goal to home")
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