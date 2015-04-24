#! /usr/bin/env python

"""
    Mocks for the tests.
"""

from random import randint, random
import rospy
from rospy import loginfo, sleep, Publisher, Subscriber, Timer, Duration
import roslib
roslib.load_manifest('pandora_fsm')
from std_msgs.msg import String

from actionlib import SimpleActionServer as ActionServer
from pandora_fsm import topics

# Messages
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from pandora_navigation_msgs.msg import DoExplorationAction
from pandora_end_effector_planner.msg import MoveEndEffectorAction
from pandora_end_effector_planner.msg import MoveLinearAction
from move_base_msgs.msg import MoveBaseAction
from pandora_data_fusion_msgs.msg import WorldModelMsg, VictimInfoMsg, DeleteVictimAction
from pandora_rqt_gui.msg import ValidateVictimGUIAction


def create_pose():
    msg = Pose()
    msg.position = create_point()
    msg.orientation = create_quaternion()

    return msg


def create_point():
    msg = Point()
    msg.x = random() * 10
    msg.y = random() * 10
    msg.z = random() * 10

    return msg


def create_quaternion():
    msg = Quaternion()
    msg.x = random() * 10
    msg.y = random() * 10
    msg.z = random() * 10
    msg.w = random() * 10

    return msg


def create_pose_stamped():
    msg = PoseStamped()
    msg.header = rospy.Header()
    msg.pose = create_pose()

    return msg


def create_victim_info(id=None, victim_frame_id=None, sensors=None, valid=None,
                       probability=None):
    msg = VictimInfoMsg()
    if id is None:
        msg.id = randint(0, 100)
    else:
        msg.id = id

    if not victim_frame_id:
        msg.victimFrameId = 'kinect'
    else:
        msg.victimFrameId = victim_frame_id

    if not sensors:
        msg.sensors = ['thermal', 'kinect']
    else:
        msg.sensors = sensors

    if valid is None:
        msg.valid = True
    else:
        msg.valid = valid

    if probability is None:
        msg.probability = random()
    else:
        msg.probability = probability
    msg.victimPose = create_pose_stamped()
    return msg


class MockActionServer(object):

    """Docstring for MockActionServer. """

    def __init__(self, name, topic, action_type):
        """ Creating a custom mock action server."""

        self._topic = topic
        self._name = name
        self._action_type = action_type
        self.timeout = 5

        rospy.Subscriber('mock/' + name, String, self.receive_commands)
        self._server = ActionServer(self._topic, self._action_type,
                                    self.success, False)
        self._server.start()
        loginfo('+ Starting ' + self._name)

    def receive_commands(self, msg):
        """ Decides the result of the next call. """

        result, timeout = msg.data.split(':')
        self.timeout = float(timeout)
        self._server.execute_callback = getattr(self, result)
        loginfo(self._name + ': Current callback -> ' + result)
        sleep(1)

    def abort(self, goal):
        """ Aborts any incoming goal. """

        loginfo(self._name + ': This goal will be aborted.')
        sleep(self.timeout)
        self._server.set_aborted()

    def success(self, goal):
        """ Succeeds any incoming goal. """

        loginfo(self._name + ': This goal will succeed.')
        sleep(self.timeout)
        self._server.set_succeeded()

    def preempt(self, goal):
        """ Preempts any incoming goal. """

        loginfo(self._name + ': This goal will be preempted.')
        sleep(self.timeout)
        self._server.set_preempted()


class WorldModel(object):

    def __init__(self, name):
        self._name = name
        self._pub = Publisher(topics.world_model, WorldModelMsg)
        Subscriber('mock/' + self._name, String, self.receive_commands)
        Subscriber('mock/victim_probability', String, self.receive_probability)
        loginfo('+ Starting ' + self._name)

    def receive_commands(self, msg):
        self.frequency = float(msg.data)
        self._rate = rospy.Rate(self.frequency)
        self.send()

    def send(self):
        while not rospy.is_shutdown():
            msg = self.create_msg()
            self._pub.publish(msg)
            self._rate.sleep()

    def create_msg(self):
        msg = WorldModelMsg()
        msg.victims = [create_victim_info() for i in range(randint(0, 3))]
        msg.visitedVictims = [create_victim_info() for i in range(randint(0, 3))]

        return msg

    def receive_probability(self, msg):
        ident_temp, prob_temp = msg.data.split(':')
        ident = int(ident_temp)
        prob = float(prob_temp)
        self.publish_custom_msg(id=ident, probability=prob)

    def publish_custom_msg(self, id=None, victim_frame_id=None, sensors=None,
                           valid=None, probability=None):
        msg = WorldModelMsg()
        msg.victims = [create_victim_info(id, victim_frame_id, sensors, valid,
                       probability)]
        rate = rospy.Rate(5)
        timeout_thershold = 7
        self._rate = rate
        self.flag = True

        # We set the threshold and the rate so that the world model is
        # updated in time for the tests to run smoothly
        Timer(Duration(timeout_thershold), self.publish_timeout, True)
        while self.flag:
            self._pub.publish(msg)
            self._rate.sleep()

    def publish_timeout(self, event):
        loginfo("finished publishing")

        self.flag = False

if __name__ == '__main__':

    rospy.init_node('mock_node')
    effector = MockActionServer('effector', topics.move_end_effector_planner,
                                MoveEndEffectorAction)
    effector = MockActionServer('linear', topics.linear_movement,
                                MoveLinearAction)
    explorer = MockActionServer('explorer', topics.do_exploration,
                                DoExplorationAction)
    move_base = MockActionServer('move',
                                 topics.move_base,
                                 MoveBaseAction)
    validate_victim_gui = MockActionServer('validate_gui',
                                           topics.gui_validation,
                                           ValidateVictimGUIAction)
    delete_victim = MockActionServer('delete_victim',
                                     topics.delete_victim,
                                     DeleteVictimAction)
    world = WorldModel('world_model')

    rospy.spin()
