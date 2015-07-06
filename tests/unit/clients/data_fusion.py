#!/usr/bin/env python

"""
    Unit tests for the DataFusion action client.
"""

import unittest

import roslib
roslib.load_manifest('pandora_fsm')

from pandora_fsm.clients import DataFusion


class TestDeleteVictim(unittest.TestCase):

    def setUp(self):
        self.df = DataFusion()

    def test_classification(self):
        res = self.df.classify_target(True, True)
        self.assertEqual(res, 'TRUE POSITIVE')
        res = self.df.classify_target(False, False)
        self.assertEqual(res, 'TRUE NEGATIVE')
        res = self.df.classify_target(False, True)
        self.assertEqual(res, 'FALSE POSITIVE')
        res = self.df.classify_target(True, False)
        self.assertEqual(res, 'FALSE NEGATIVE')
