#!/usr/bin/env python

"""
    Unit tests for the DataFusion action client.
"""

import unittest

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from pandora_fsm.clients import DataFusion
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus


class TestDeleteVictim(unittest.TestCase):

    def setUp(self):
        rospy.init_node('data_fusion_client_test')
        self.mock_deletion = rospy.Publisher('/mock/delete_victim', String)
        self.df = DataFusion()

    def test_victim_deletion_success(self):
        if not rospy.is_shutdown():
            rospy.sleep(1)
            self.mock_deletion.publish('success:1')
        self.df.delete_victim(2)

        self.assertTrue(self.df.deletion.get_state(), GoalStatus.SUCCEEDED)

    def test_victim_deletion_abort(self):
        if not rospy.is_shutdown():
            rospy.sleep(1)
            self.mock_deletion.publish('abort:1')
        self.df.delete_victim(2)

        self.assertTrue(self.df.deletion.get_state(), GoalStatus.SUCCEEDED)

    def test_classification(self):
        res = self.df.classify_target(True, True)
        self.assertEqual(res, 'TRUE POSITIVE')
        res = self.df.classify_target(False, False)
        self.assertEqual(res, 'TRUE NEGATIVE')
        res = self.df.classify_target(False, True)
        self.assertEqual(res, 'FALSE POSITIVE')
        res = self.df.classify_target(True, False)
        self.assertEqual(res, 'FALSE NEGATIVE')
