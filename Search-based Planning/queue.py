#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@author: Huiming Zhou
@description: this file defines three kinds of queues that will be used in algorithms.
"""

import collections
import heapq


"""
    Class: QueueFIFO
    Description: QueueFIFO is designed for First-in-First-out rule.
"""


class QueueFIFO:
    def __init__(self):
        self.queue = collections.deque()

    def empty(self):
        return len(self.queue) == 0

    def put(self, node):
        self.queue.append(node)            # enter from back

    def get(self):
        return self.queue.popleft()        # leave from front


"""
    Class: QueueLIFO
    Description: QueueLIFO is designed for Last-in-First-out rule.
"""


class QueueLIFO:
    def __init__(self):
        self.queue = collections.deque()

    def empty(self):
        return len(self.queue) == 0

    def put(self, node):
        self.queue.append(node)            # enter from back

    def get(self):
        return self.queue.pop()            # leave from back


"""
    Class: QueuePrior
    Description: QueuePrior reorders elements using value [priority]
"""


class QueuePrior:
    def __init__(self):
        self.queue = []

    def empty(self):
        return len(self.queue) == 0

    def put(self, item, priority):
        heapq.heappush(self.queue, (priority, item))    # reorder x using priority

    def get(self):
        return heapq.heappop(self.queue)[1]             # pop out the smallest item
