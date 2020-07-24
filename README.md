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
            ├── LRTAstar.py                             # Learning Real-time A*
            ├── RTAAstar.py                             # Real-time Adaptive A*
            ├── LPAstar.py                              # Lifelong Planning A*
            ├── D_star.py                               # D* (Dynamic A*)
            ├── Anytime_D_star.py                       # Anytime D*
            └── D_star_Lite.py                          # D* Lite
        └── Search_3D
            ├── Astar3D.py                              # A*_3D
            ├── bidirectional_Astar3D.py                # Bidirectional A*_3D
            ├── RTA_Astar3D.py                          # Real-time Adaptive A*_3D
            └── LRT_Astar3D.py                          # Learning Real-time A*_3D
    └── Sampling-based Planning
        └── rrt_2D
            ├── rrt.py                                  # rrt : goal-biased rrt
            └── rrt_star.py
        └── rrt_3D
            ├── rrt3D.py                                # rrt3D : goal-biased rrt3D
            └── rrtstar3D.py
    └── Stochastic Shortest Path
        ├── value_iteration.py                          # value iteration
        ├── policy_iteration.py                         # policy iteration
        ├── Q-value_iteration.py                        # Q-value iteration
        └── Q-policy_iteration.py                       # Q-policy iteration
    └── Model-free Control
        ├── Sarsa.py                                    # SARSA : on-policy TD control
        └── Q-learning.py                               # Q-learning : off-policy TD control

## Animations - Search-Based
### Best-First & Dijkstra
<div align=right>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/BF.gif" alt="dfs" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/Dijkstra.gif" alt="dijkstra" width="400"/></a></td>
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
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/RepeatedA_star.gif" alt="repeatedastar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/ARA_star.gif" alt="arastar" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/LRTA_star.gif" alt="lrtastar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/RTAA_star.gif" alt="rtaastar" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/LPAstar.gif" alt="lpastar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/D_star_Lite.gif" alt="dstarlite" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/ADstar_small.gif" alt="lpastar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search-based%20Planning/gif/ADstar_sig.gif" alt="dstarlite" width="400"/></a></td>
  </tr>
</table>
</div>

## Animation - Sampling-Based
### RRT & Variants
<div align=right>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/PathPlanning/blob/master/Sampling-based%20Planning/gif/RRT_2D.gif" alt="value iteration" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/PathPlanning/blob/master/Sampling-based%20Planning/gif/RRT_CONNECT_2D.gif" alt="value iteration" width="400"/></a></td>
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

## Papers
### Search-base Planning
* [D*: ](http://web.mit.edu/16.412j/www/html/papers/original_dstar_icra94.pdf) Optimal and Efficient Path Planning for Partially-Known Environments
* [Lifelong Planning A*: ](https://www.cs.cmu.edu/~maxim/files/aij04.pdf) Lifelong Planning A*
* [Anytime Repairing A*: ](https://papers.nips.cc/paper/2382-ara-anytime-a-with-provable-bounds-on-sub-optimality.pdf) ARA*: Anytime A* with Provable Bounds on Sub-Optimality
* [D* Lite: ](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf) D* Lite
* [Field D*: ](http://robots.stanford.edu/isrr-papers/draft/stentz.pdf) Field D*: An Interpolation-based Path Planner and Replanner
* [Anytime D*: ](http://www.cs.cmu.edu/~ggordon/likhachev-etal.anytime-dstar.pdf) Anytime Dynamic A*: An Anytime, Replanning Algorithm
* [Focussed D*: ](http://robotics.caltech.edu/~jwb/courses/ME132/handouts/Dstar_ijcai95.pdf) The Focussed D* Algorithm for Real-Time Replanning
* [Potential Field, ](https://journals.sagepub.com/doi/abs/10.1177/027836498600500106) [[PPT]: ](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf) Real-Time Obstacle Avoidance for Manipulators and Mobile Robots
* [Hybrid A*: ](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf) Practical Search Techniques in Path Planning for Autonomous Driving

### Sampling-based Planning
* [RRT: ](http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf) Rapidly-Exploring Random Trees: A New Tool for Path Planning
* [RRT-Connect: ](http://www-cgi.cs.cmu.edu/afs/cs/academic/class/15494-s12/readings/kuffner_icra2000.pdf) RRT-Connect: An Efficient Approach to Single-Query Path Planning
* [Extended-RRT: ](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.1.7617&rep=rep1&type=pdf) Real-Time Randomized Path Planning for Robot Navigation
* [Dynamic-RRT: ](https://www.ri.cmu.edu/pub_files/pub4/ferguson_david_2006_2/ferguson_david_2006_2.pdf) Replanning with RRTs
