Mix_initiative Control
========

Human-in-the-loop mix initiative control under temporal tasks

```
@INPROCEEDINGS{8460793,
  author={M. {Guo} and S. {Andersson} and D. V. {Dimarogonas}},
  booktitle={2018 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Human-in-the-Loop Mixed-Initiative Control Under Temporal Tasks}, 
  year={2018},
  volume={},
  number={},
  pages={6395-6400},
  doi={10.1109/ICRA.2018.8460793}}
```

-----
Description
-----
This package contains the implementation of the mix-initiative control of a single robot under temporal tasks. The human operator can directly modify the navigation input of the robot and assign new tasks to the robot during run time. The workspace is assumed to be only partially-known and possibly dynamic. More importantly, via this interaction, the robot can learn the human preference for the parameters used in the plan synthesis.

For simulation files based on [TIAGo robot](http://wiki.ros.org/Robots/TIAGo), see [**`/tiago`**](https://github.com/MengGuo/mix_initiative/tree/master/tiago).

For experiment files based on [TurtleBot](https://github.com/MengGuo/mix_initiative/tree/master/turtlebot), see [**`/turtlebot`**](https://github.com/MengGuo/mix_initiative/tree/master/turtlebot).

For the complete ROS package, see [**`/hil_mix_control`**](https://github.com/MengGuo/mix_initiative/tree/master/hil_mix_control)


<p align="center">
  <img src="https://github.com/MengGuo/mix_initiative/blob/master/hil_mix_control/src/figures/combined.png" width="700"/>
</p>


-----
Features
-----
- Human operator can influence the `cmd_vel` control velocities whenever needed:

  -- to guide the robot through unknown area of the workspace,
  
  -- to show the preferred path.
  
- Safety is ensured for all time by the mix-initiative controller, for all possible human inputs. 
- Human can assign contingent short-term tasks during run time, which the robot will accommodate within the given deadline.


-----
Content
-----
* Mix-initiative controller. 

  ```python
    MixPublisher = rospy.Publisher('mix_vel', Twist, queue_size = 100)
    # control command from amcl navigation
    rospy.Subscriber('nav_vel', Twist, NaviControlCallback)
    # control command from tele operation
    rospy.Subscriber('key_vel', Twist, TeleControlCallback)
	mix_control, gain = smooth_mix(tele_control, navi_control, dist_to_trap)
	SendMix(MixPublisher, mix_control)
	print 'mix_control: %s ||| navi_control: %s ||| tele_control: %s ||| gain: %.2f' %(mix_control, navi_control, tele_control, gain)
  ```
  The [**`/tiago/twist_mux_topics.yaml`**](https://github.com/MengGuo/mix_initiative/blob/master/tiago/twist_mux_topics.yaml) needs to be modified by: 1. add `mix_vel` to the receiving topics; 2. give `mix_vel` much higher priority than `nav_vel` and `joy_vel`.

<p align="center">
  <img src="https://github.com/MengGuo/mix_initiative/blob/master/hil_mix_control/src/figures/v_1.png" width="700"/>
</p>

<p align="center">
  <img src="https://github.com/MengGuo/mix_initiative/blob/master/hil_mix_control/src/figures/zoom_v_1.png" width="700"/>
</p>


* Inverse reinforcement learning (IRL)

  Given the past inputs from the human, the robot could learn the preferred value of the parameters used in the plan synthesis. This is closely related to the [inverse reinforcement learning (IRL) problem](http://ai.stanford.edu/~ang/papers/icml00-irl.pdf), where the robot learns about the cost functions of the system model based on demonstration of the preferred plans. On the other hand, in [reinforcement learning](http://incompleteideas.net/sutton/book/ebook/the-book.html) problem, the robot learns the optimal plan given these functions.

  Most problems of IRL are ill-posed. In order to improve **generalization** such that the robot could infer the human preference based on the human's past inputs (instead of simply repeating them), our solution is based on the [maximum margin planning algorithm](http://dl.acm.org/citation.cfm?id=1143936).

```python
	print 'Iteration --%d--'%count
	beta = beta_p
	marg_path = self.margin_opt_path(opt_path, beta)
	marg_cost = self.compute_path_cost(marg_path)
	marg_ac_d = marg_cost[1]
	print '(opt_ac_d-marg_ac_d)', opt_ac_d-marg_ac_d
	#gradient = beta + lam*(opt_ac_d-marg_ac_d)
	gradient = lam*(opt_ac_d-marg_ac_d)
	if count <10:
			beta_p = beta - (alpha)*gradient
	else:
			beta_p = beta - (alpha/(count+1))*gradient
	print 'gradient:%.2f and beta_dif:%.2f' %(gradient, beta-beta_p)
	count += 1
	print 'old beta: %.2f ||| new beta: %.2f' %(beta, beta_p)
	score = self.opt_path_match(opt_path, marg_path)
```

<p align="center">
  <img src="https://github.com/MengGuo/mix_initiative/blob/master/hil_mix_control/src/figures/beta.png" width="700"/>
</p>

  As a result, the robot's discrete plan can be updated based on the learned parameter. It can be seen below that the human only needs to interfere the robot's motions when needed.

<p align="center">
  <img src="https://github.com/MengGuo/mix_initiative/blob/master/hil_mix_control/src/figures/traj_1.png" width="700"/>
</p>



* Temporary Task Assignment

  The human can assign a temporary task to the robot during run time, with a preferred deadline. The adaptation algorithm [**`add_temp_task`**](https://github.com/MengGuo/mix_initiative/blob/master/hil_mix_control/src/ltl_tools/discrete_plan.py) finds the optimal indices where the robot could derived from its current plan and satisfy the temporary task.
  
<p align="center">
  <img src="https://github.com/MengGuo/mix_initiative/blob/master/hil_mix_control/src/figures/traj_2.png" width="700"/>
</p>


-----
Installation
-----

* TIAGo simulation repo: http://wiki.ros.org/Robots/TIAGo/Tutorials

* LTL motion planner: https://github.com/MengGuo/P_MAS_TG

* TurtleBot repo: http://wiki.ros.org/Robots/TurtleBot
