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

    def put_set(self, dictin):
        '''add a new dict into the priority queue'''
        for item, priority in enumerate(dictin):
            self.put(item, priority)

    def check_remove(self, item):
        if item not in self.entry_finder:
            return
        entry = self.entry_finder.pop(item)
        entry[-1] = self.REMOVED
        self.nodes.remove(item)

    def check_remove_set(self, set_input):
        if len(set_input) == 0:
            return
        for item in set_input:
            if item not in self.entry_finder:
                continue
            entry = self.entry_finder.pop(item)
            entry[-1] = self.REMOVED
            self.nodes.remove(item)

    def priority_filtering(self, threshold, mode):
        # mode: bigger: check and remove those key vals bigger than threshold
        if mode == 'lowpass':
            for entry in self.enumerate():
                item = entry[2]
                if entry[0] >= threshold: # priority
                    _ = self.entry_finder.pop(item)
                    entry[-1] = self.REMOVED
                    self.nodes.remove(item)
        # mode: smaller: check and remove those key vals smaller than threshold
        elif mode == 'highpass':
            for entry in self.enumerate():
                item = entry[2]
                if entry[0] <= threshold: # priority
                    _ = self.entry_finder.pop(item)
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
