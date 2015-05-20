#!/usr/bin/env python

""" Tests for the MultipleEvent class """


import unittest
import time
from threading import Thread, Event

from pandora_fsm.utils import MultipleEvent


class TestMultipleEvent(unittest.TestCase):

    def setUp(self):

        # Create some events to wait for
        self.e1 = Event()
        self.e2 = Event()
        self.e3 = Event()
        self.e4 = Event()

    def sender(self, event, delay):

        time.sleep(delay)
        event.set()

    def test_two_events(self):

        events = {'e1': self.e1,
                  'e2': self.e2}

        multi = MultipleEvent(events)
        Thread(target=self.sender, args=(self.e1, 2,)).start()
        Thread(target=self.sender, args=(self.e2, 4,)).start()

        multi.wait()

        self.assertTrue(self.e1.is_set())
        self.assertFalse(self.e2.is_set())

    def test_four_events(self):

        events = {'e1': self.e1,
                  'e2': self.e2,
                  'e3': self.e3,
                  'e4': self.e4}

        multi = MultipleEvent(events)
        Thread(target=self.sender, args=(self.e1, 5,)).start()
        Thread(target=self.sender, args=(self.e2, 3,)).start()
        Thread(target=self.sender, args=(self.e3, 7,)).start()
        Thread(target=self.sender, args=(self.e4, 5,)).start()

        multi.wait()

        self.assertTrue(self.e2.is_set())
        self.assertFalse(self.e1.is_set())
        self.assertFalse(self.e3.is_set())
        self.assertFalse(self.e4.is_set())


if __name__ == '__main__':
    unittest.main()
