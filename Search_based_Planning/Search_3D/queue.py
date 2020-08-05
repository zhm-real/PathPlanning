import collections
import heapq
import itertools

class QueueFIFO:
    """
    Class: QueueFIFO
    Description: QueueFIFO is designed for First-in-First-out rule.
    """

    def __init__(self):
        self.queue = collections.deque()

    def empty(self):
        return len(self.queue) == 0

    def put(self, node):
        self.queue.append(node)  # enter from back

    def get(self):
        return self.queue.popleft()  # leave from front


class QueueLIFO:
    """
    Class: QueueLIFO
    Description: QueueLIFO is designed for Last-in-First-out rule.
    """

    def __init__(self):
        self.queue = collections.deque()

    def empty(self):
        return len(self.queue) == 0

    def put(self, node):
        self.queue.append(node)  # enter from back

    def get(self):
        return self.queue.pop()  # leave from back


class QueuePrior:
    """
    Class: QueuePrior
    Description: QueuePrior reorders elements using value [priority]
    """

    def __init__(self):
        self.queue = []

    def empty(self):
        return len(self.queue) == 0

    def put(self, item, priority):
        heapq.heappush(self.queue, (priority, item))  # reorder s using priority

    def get(self):
        return heapq.heappop(self.queue)[1]  # pop out the smallest item

    def enumerate(self):
        return self.queue

    def check_remove(self, item):
        for (p, x) in self.queue:
            if item == x:
                self.queue.remove((p, x))

    def top_key(self):
        return self.queue[0][0]

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

# class QueuePrior:
#     """
#     Class: QueuePrior
#     Description: QueuePrior reorders elements using value [priority]
#     """

#     def __init__(self):
#         self.queue = []

#     def empty(self):
#         return len(self.queue) == 0

#     def put(self, item, priority):
#         count = 0
#         for (p, s) in self.queue:
#             if s == item:
#                 self.queue[count] = (priority, item)
#                 break
#             count += 1
#         if count == len(self.queue):
#             heapq.heappush(self.queue, (priority, item))  # reorder s using priority

#     def get(self):
#         return heapq.heappop(self.queue)[1]  # pop out the smallest item

#     def enumerate(self):
#         return self.queue

#     def check_remove(self, item):
#         for (p, s) in self.queue:
#             if item == s:
#                 self.queue.remove((p, s))

#     def top_key(self):
#         return self.queue[0][0]