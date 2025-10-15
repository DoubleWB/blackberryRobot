This readme has been added as part of an initial commit of a lot of work to a github repository. 

This project serves as an overall rewrite and redesign of the code I wrote to control Blueberry, a 5 DOF robot that was the predecessor to Blackberry.

As this project moves closer to a final form, more implementation details to come - for the time being this shall serve as a documentation of the work currently done and the future work that might define this project.

The initial commit represents the work that was done in one burst over parental leave - it includes:
 * Code to communicate with and control Dynamixel motors
 * Code to organize Dynamixel motors into joints that are part of one robot arm
 * Code to allow the reconfiguration or tweaking of the robot arm via config file
 * Code to run forward kinematics on the blackberry robot arm

Future additions will include:
 * Code to enforce that the arm will stop ongoing movement commands if failures are detected in any joint
 * Code to work around extended position control positions being reset upon arm reboot
 * Inverse kinematics on the blackberry robot arm
  * Orientation and Translation based inverse kinematics separately
 * Integration with some sort of networked solution (ROS?) to allow for the robot to receive teleoperation commands
 * Trajectory planning
 * Static (environment based) object avoidance
 * Dynamic object avoidance
