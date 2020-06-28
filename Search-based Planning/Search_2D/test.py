"""
A_star 2D
@author: huiming zhou
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import queue

q = queue.QueuePrior()
q.put((1, 2), 3)
print(q.enumerate())
q.put((1, 2), 2)
print(q.enumerate())
q.put((1, 2), 4)
print(q.enumerate())
