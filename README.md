# isaac_cv_ros 

**isaac_cv_ros** is a package to allow ROS based simulation of a MAV equipped with a 3D-reconstruction sensor. The simulation is performed inside an [Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/reference_python_api.html) environment. The node-game communcation is carried out utilizing the [ROS bridge](https://docs.omniverse.nvidia.com/isaacsim/latest/install_ros.html) plugin for Isaac Sim.


# Table of Contents
**Installation**
* [Dependencies](#Dependencies)
* [Installation](#Installation)

**Examples**
* [Run in test mode](#Run-in-test-mode)
* [Run with MAV](#Run-with-MAV)

**Documentation: ROS Nodes**
* [unreal_ros_client](#unreal_ros_client)
* [sensor_model](#sensor_model)
* [simulation_manager](#simulation_manager)


# Installation
## Dependencies

**ROS Packages:**

The perception functionalities (unreal_ros_client + sensor_model) depend on:
* `catkin_simple` ([https://github.com/catkin/catkin_simple](https://github.com/catkin/catkin_simple))

To run the full MAV simulation, these additional packages are needed: 
* `rotors_simulator` ([https://github.com/ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator))
* `mav_control_rw` ([https://github.com/ethz-asl/mav_control_rw](https://github.com/ethz-asl/mav_control_rw))
* `voxblox` ([https://github.com/ethz-asl/voxblox](https://github.com/ethz-asl/voxblox))

## Installation
**Install the ROS package:**

Installation instructions on Linux:

1. Move to your catkin workspace: 
```shell script
cd ~/catkin_ws/src
```
2. Install using a SSH key or via HTTPS: 
```shell script
git clone git@github.com:ethz-asl/unreal_cv_ros.git # SSH
git clone https://github.com/ethz-asl/unreal_cv_ros.git # HTTPS
```
3. Compile: 
```shell script
catkin build unreal_cv_ros
```
**Install Isaac Sim:**

TBD


## Data Repository
TBD

# Examples
## Run in test mode
TBD

## Run with MAV
TBD

# Documentation: ROS Nodes
## isaac_ros_client
This node manages the Isaac Sim client and the connection with a Isaac Sim environment. It sets the camera position and orientation within the Isaac Sim environment and produces the raw image data and camera calibration used by the sensor_model.

### Parameters
  
* **collision_on** Set to true to check for collision in the Isaac Sim. Set to false to set the camera anyway. May result in rendering artifacts if the camera overlaps with objects. Default is true.
* **collision_tol** This parameter only shows in `standard` mode. Collision warnings are triggered if the requested and realized position are further apart than this threshold (in unreal units, default unit is cm). Default is 10.
* **publish_tf** If true, the client pulishes a tf-transform of the camera pose for every taken image with a matching timestamp. Default is False.
* **slowdown** Artificially slows down the time between setting the pose and taking images to give unreal engine more time to render the new view. Slowdown is expected as wait duration in seconds wall-time. Default is 0.0.
* **camera_id** Lets unrealcv know which camera to use. Default is 0.
* **queue_size** Queue size of the input subscriber. Default is 1. Leads to a "as fast as possible" image gathering without delay.
<!-- TODO: (michbaum) Can be received directly from the Simulation and published. -->
* **Camera Parameters:** To change the resolution and field of view (FOV) of the camera, the [unrealcv configuration file](http://docs.unrealcv.org/en/master/plugin/config.html) needs to be changed. The relevant path is displayed when the unreal_ros_client node is launched. When the client is setup, these values are published as 'camera_params' on the ros parameter server for other nodes to access them.

### Input Topics
* **odometry** of type `nav_msgs.msg/Odometry`. Set the camera pose w.r.t. its position and yaw at the connection of the client.

### Output Topics
<!-- TODO: (michbaum) Change to normal image types. -->
* **ue_sensor_raw** of type `unreal_cv_ros.msg/UeSensorRaw`. The output of the in-game image capture, containing a color and a depth image encoded as npy binaries.
* **collision** of type `std_msgs.msg/String`. Publishes "MAV collision detected!" upon collision detection. Only available if collision_on is true.

### Services
* **terminate_with_reset** of type `std_srvs.srv/SetBool`. Stop processing new odom requests and reset the camera to the initial pose.

## sensor_model
<!-- TODO: (michbaum) Also probably does need change. -->
This node converts the unreal_ros_client output into a pointcloud for further processing. Sensor specific behaviour can be simulated by artificially altering the ground truth data.

### Parameters
* **model_type** Which sensor to simulate. Currently implemented are: 
  * **ground_truth:** Produces the ground truth pointcloud without additional processing.
  * **kinect:** Simulates a Kinect 3D sensor according to [this paper](https://ieeexplore.ieee.org/abstract/document/6375037). Notice that the sensor range is cropped to \[0.5, 3.0\] m and that the inclination angle is neglected, since it is constant up until ~60, 70 degrees.
  * **gaussian_depth_noise:** Apply a gaussian, depth dependent term to the pointcloud z coordinate. Allows to set the params `k_mu_<i>` and `k_sigma_<i>` for i in \[0, 3\], where f(z) = k0 + k1 * z + k2 * z^2 + k3 * z^3. Default for all k is 0.0.
  
  Default is 'ground_truth'.
* **camera_params_ns** Namespace where to read the Isaac Sim camera parameters from, which are expected as {height, width, focal_length}. Notice that the sensor_model waits until the camera params are set on the ros parameter server (e.g. from the isaac\_ros\_client). Default is 'isaac\_ros\_client/camera_params'.
* **maximum_distance** All points whose original ray length is beyond maximum_distance are removed from the pointcloud. Set to 0 to keep all points. Default is 0.0.
* **flatten_distance** Sets the ray length of every point whose ray length is larger than flatten\_distance to flatten\_distance. Set to 0 to keep all points unchanged. Default is 0.0.
* **publish_color_images** In addition to the point clouds publish the perceived images, encoded as 4 channel RGBA. Default is False.
* **publish_gray_images** If true publish a gray scale image with every pointcloud (e.g. for integration with rovio). Default is False.

<!-- TODO: (michbaum) These topic names need to be changed. -->
### Input Topics
* **ue_sensor_raw** of type `unreal_cv_ros.msg/UeSensorRaw`. Output of the unreal\_ros\_client that is to be further processed.

### Output Topics
* **ue_sensor_out** of type `sensor_msgs.msg/PointCloud2`. Resulting pointcloud after applying the simulated sensing pipeline.
* **ue_color_image_out** of type `sensor_msgs.msg/Image`. Color image of the current view. Only published if publish_color_images is true.
* **ue_gray_image_out** of type `sensor_msgs.msg/Image`. Gray scale image of the current view. Only published if publish_gray_images is true.


## simulation_manager
This node is used to launch the full MAV simulation using gazebo as a physics engine and an isaac sim game for perception and collision modeling. It is used to coordinate simulation setup and monitor the isaac_cv_ros pipeline performance.

### Parameters
* **ns_gazebo** Namespace of gazebo, including the node name. Default is '/gazebo'.
* **ns_mav** Namespace of the MAV, which is expected to end with the actual MAV name. Default is '/firefly'.
* **monitor** Set to true to measure the Isaac Sim vision pipeline's performance. Default is False.
* **horizon** How many datapoints are kept and considered for the performance measurement. Only available if monitor is true. Default is 10.

### Input Topics
<!-- TODO: (michbaum) Needs to change. -->
* **ue_raw_in** of type `unreal_cv_ros.msg/UeSensorRaw`. Output of the isaac\_ros\_client for performance measurements. Only available if monitor is true.
* **ue_out_in** of type `sensor_msgs.msg/PointCloud2`. Output of the sensor model for performance measurements. This topic needs to be matched to check for correct startup.

### Output Topics
* **simulation_ready** of type `std_msgs.msg/String`. After successful start of the pipeline publishes "Simulation Ready" to let other nodes start or take over.

### Services
* **display_monitor** of type `std_srvs.srv/Empty`. Print the current monitoring measurements to console. Only available if monitor is true.

