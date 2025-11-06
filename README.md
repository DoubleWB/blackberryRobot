# The Blackberry Robot
The Blackberry robot is my personal 6 DOF robotic manipulator based on the open source Trossen WidowX 250S. It is slightly smaller, and uses improvised gear boxes to increase the torque of lower output motors, but is generally the same form factor.

# The Design
The only truly novel part of the Blackberry physical design is the decorative motor board enclosure, and the addition of a gear box to the elbow and shoulder flexion joints. 3D Files for these parts will be attached once the designs are finalized or close to it.

# The Software
The aim of this project is to build understanding about robotic manipulators by trying to build a robust platform more or less from scratch. Some of the code base is built off the software to drive the Blueberry - a 5DOF arm that was the predecessor to the Blackberry.

## Current Functionality
### Low level control
 * Easy configuration of the Dynamixel motors tuning parameters and mounting configuration via config file.
 * Individual, joint, and robot level control of the Dynamixel motors.
 * Collision/stress protected movements, to avoid the robot breaking itself or injuring any human collaborators (the aim is for this manipulator to be a Co-bot).
 * Calibration routines for non trivial joints to ensure consistent startup behavior.
### Kinematics
 * Efficient forward kinmatics mapping joint angles to X, Y, Z, Yaw, Pitch, Roll definied poses.
 * Basic inverse kinematics control via gradient descent. Seems to function well when well within the workspace of the manipulator.

## Wishlist/Future topics of development
### Safety
 * Robustification against self collision in otherwise valid joint configurations.
 * Robustification against external collision with pre-defined obstacles in otherwise valid joint configurations.
 * System of dynamic obstacle avoidance.
 * Sensor based input to the above dynamic system.

### Kinematics
 * General performance improvement of inverse kinematics.
  * Cacheing of previously achieved poses.
  * Experimenting with Damped Least Squares to improve speed at the cost of accuracy.
  * Early exit conditions to better detect local minima.
 * Reasonable implementation of translation only/orientation only inverse kinematics.
 * Trajectory planning.
 * Partial Arm Kinematics.

### Ease of Use
 * Handheld pose teaching by a human collaborator.
 * More robust calibration routines that are safe against collision from any starting configuration.
 * Robot control over ROS.
 * Better input devices for robot control than inputting a pose numerically.

### Behaviors
 * Some hardcoded behavior scripts like waving or interacting with pre placed objects.
 * The ability to point the manipulator at a point in space, and mapping those points to human identifiers.
 * Autonomous routines, such as dynamic grasping or pick and place.

# Status!
This README has been rewritten to reflect that the project has reached a true 'starting point' - enough functionality is present to begin experimenting with and researching different robotic manipulator capabilities. Next steps will involve setting up a permanent home for the Blackberry, and then picking some items off of the long wishlist to start implementing!