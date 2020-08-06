# min heap used in the FMT*

import collections
import heapq
import itertools

class MinheapPQ:
    """
    A priority queue based on min heap, which takes O(logn) on element removal
    https://docs.python.org/3/library/heapq.html#priority-queue-implementation-notes
    """
    def __init__(self):
        self.pq = [] # lis of the entries arranged in a heap
        self.nodes = set()
        self.entry_finder = {} # mapping of the item entries
        self.counter = itertools.count() # unique sequence count
        self.REMOVED = '<removed-item>'
    
    def put(self, item, priority):
        '''add a new task or update the priority of an existing item'''
        if item in self.entry_finder:
            self.check_remove(item)
        count = next(self.counter)
        entry = [priority, count, item]
        self.entry_finder[item] = entry
        heapq.heappush(self.pq, entry)
        self.nodes.add(item)

    def check_remove(self, item):
        if item not in self.entry_finder:
            return
        entry = self.entry_finder.pop(item)
        entry[-1] = self.REMOVED
        self.nodes.remove(item)

    def get(self):
        """Remove and return the lowest priority task. Raise KeyError if empty."""
        while self.pq:
            priority, count, item = heapq.heappop(self.pq)
            if item is not self.REMOVED:
                del self.entry_finder[item]
                self.nodes.remove(item)
                return item
        raise KeyError('pop from an empty priority queue')

    def top_key(self):
        return self.pq[0][0]
        
    def enumerate(self):
        return self.pq

    def allnodes(self):
        return self.nodes
