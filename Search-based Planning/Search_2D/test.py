"""
A_star 2D
@author: huiming zhou
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import queue
from Search_2D import plotting
from Search_2D import env


U = queue.QueuePrior()
U.put((1, 2), [2, 3])
U.put((2, 3), [1, 5])
print(U.get())