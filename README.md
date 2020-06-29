Directory Structure
------
    .
    └── Search-based Planning
        └── Search_2D
            ├── bfs.py                                  # breadth-first searching
            ├── dfs.py                                  # depth-first searching
            ├── dijkstra.py                             # dijkstra's
            ├── a_star.py                               # A*
            ├── bidirectional_a_star.py                 # Bidirectional A*
            ├── ARAstar.py                              # Anytime Reparing A*
            ├── IDAstar.py                              # Iteratively Deepening A*
            └── LRTAstar.py                             # Learning Real-time A*
        └── Search_3D
            ├── Astar3D.py                              # A*_3D
            ├── bidirectional_Astar3D.py                # Bidirectional A*_3D
            └── LRT_Astar3D.py                          # Learning Real-time A*_3D
        └── gif                                         # Animations
    └── Stochastic Shortest Path
        ├── value_iteration.py                      # value iteration
        ├── policy_iteration.py                     # policy iteration
        ├── Q-value_iteration.py                    # Q-value iteration
        └── Q-policy_iteration.py                   # Q-policy iteration
    └── Model-free Control
        ├── Sarsa.py                                # SARSA : on-policy TD control
        └── Q-learning.py                           # Q-learning : off-policy TD control
    └── Sampling-based Planning
        └── rrt_2D
            ├── rrt.py                              # rrt : goal-biased rrt
            └── rrt_star.py
        └── rrt_3D
            ├── rrt3D.py                            # rrt3D : goal-biased rrt3D
            └── rrtstar3D.py

## Animations
### DFS & BFS (Dijkstra)
* Blue: starting state
* Green: goal state

<div align=right>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/DFS.gif" alt="dfs" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/BFS.gif" alt="bfs" width="400"/></a></td>
  </tr>
</table>
</div>

### A* and A* Variants
<div align=right>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/Astar.gif" alt="astar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/Bi-Astar.gif" alt="biastar" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/ARA_star.gif" alt="arastar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/LRTA_star.gif" alt="lrtastar" width="400"/></a></td>
  </tr>
</table>
</div>

### Value/Policy/Q-value/Q-policy Iteration
* Brown: losing states
<div align=right>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Stochastic%20Shortest%20Path/gif/VI.gif" alt="value iteration" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Stochastic%20Shortest%20Path/gif/VI_E.gif" alt="value iteration" width="400"/></a></td>
  </tr>
</table>
</div>

### SARSA(on-policy) & Q-learning(off-policy)
* Brown: losing states
<div align=right>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Model-free%20Control/gif/SARSA.gif" alt="value iteration" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Model-free%20Control/gif/Qlearning.gif" alt="value iteration" width="400"/></a></td>
  </tr>
</table>
</div>

## License
MIT License
