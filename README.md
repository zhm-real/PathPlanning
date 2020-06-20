Directory Structure
------
    .
    └── Search-based Planning
        ├── bfs.py                                  # breadth-first
        ├── dfs.py                                  # depth-first
        ├── dijkstra.py                             # dijkstra's
        ├── a_star.py                               # a*
        ├── queue.py                                # FIFO, FILO, Priority queues
        ├── env.py                                  # environment: working space
        ├── motion_model.py                         # motion model, feasible input
        └── tools.py                                # animation, figure generation ...
    └── Stochastic Shortest Path
        ├── value_iteration.py                      # value iteration
        ├── policy_iteration.py                     # policy iteration
        ├── Q-value_iteration.py                    # Q-value iteration
        └── Q-policy_iteration.py                   # Q-policy iteration
    └── Model-free Control
        ├── Sarsa.py                                # SARSA : on-policy TD control
        └── Q-learning.py                           # Q-learning : off-policy TD control
    └── Sampling-based Planning

## License
MIT License
