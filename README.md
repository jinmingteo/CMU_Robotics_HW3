## Q1

![Plot](https://github.com/jinmingteo/CMU_Robotics_HW3/blob/master/code/Q1.png)

### Summary of key learnings

There is a need to play around with the num_basis to find a suitable DMP trajectory. The appropriate num_basis should be about the number of joints in the space trajectory. It seems that after a threshold, the DMP trajectories do not differ much.

### Usefulness of DMP 

DMP seems to be good for performing oscillating tasks. DMP trajectories also ensure smooth movement.

## Q2

![Video](https://github.com/jinmingteo/CMU_Robotics_HW3/blob/master/code/Q2.gif)

### RRT planner description
The planner sample possible trajectories to the goal. From the origin, the planner begin mapping random edges towards the qGoal. It then pass through path shortening, ensuring that the robot does not take too big steps.

### Takeaways
- Realized that my IK limit_angle=0.05 was too high, resulting in no feasible path to TGoal.

- with a lower randomness in TGoal, the robotics movement seems jerky.

- with more iterations of path shortening, the robot seems to be very close to the other objects but not in collision.