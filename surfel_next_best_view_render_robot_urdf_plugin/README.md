Surfel Next Best View Render Robot URDF Plugin
==============================================

When evaluating each view pose to find the NBV, the robot configuration to reach the view poses should be considered.
If the robot is between the sensor and the target region, the view pose may be occluded and should receive a lower score.
This plugin extends the `surfel_next_best_view` package by rendering the robot model in the virtual view of the sensor, to properly predict which parts of the scene would be occluded.

The plugin requires the URDF model of the robot.
For each view pose, the robot configuration is defined by the state of its joints.

The plugin is based on the `realtime_urdf_filter` package by Nico Blodow:  
``https://github.com/blodow/realtime_urdf_filter``

Related Publication
-------------------

+ R. Monica, J. Aleotti, "A 3D Robot Self Filter for Next Best View Planning", The Third IEEE International Conference on Robotic Computing (IRC 2019), February 25-27, 2019 Naples, Italy

Dependencies
------------

+ Surfel Next Best View: `https://github.com/RMonica/surfel_next_best_view`
+ ROS MoveIt! planner

Usage
-----

This plugin is loaded from the `surfel_next_best_view` node when parameter `enable_robot_filter` is set to true.

Two fields must be filled in the request to `~/evaluate_poses`:

+ **joint_states**: joint states of the robot, one for each viewpoint.
+ **robot_pose**: a `geometry_msgs/Pose`, origin of the robot model in the scene.

Parameters
----------

The following parameters can be set in the `surfel_next_best_view` node and are used by the plugin:

+ **depth_distance_threshold**: simulated measurement in front of the robot up to this distance from it are removed. Simulated measurements behind the robot are always removed.
+ **draw_robot_depth**: if true, the robot is rendered with positive depth values, so it counts as occupied surfels in the score computation (useful for debugging). If false, the robot is rendered with value 0 (out of range).
+ **models**: an array of parameters. Each array element contains a single field:
    + **model**: name of the parameter which contains the robot description, in the parameter server.
