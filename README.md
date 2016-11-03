# Online Multimodal Interactive Perception - OMIP

### Authors and contact

(Main author) Roberto Martín-Martin (roberto.martinmartin@tu-berlin.de, rmartinmar@gmail.com)

Sebastian Höfer (sebastian.hoefer@tu-berlin.de, sebastianhoefer83@googlemail.com)

Oliver Brock (oliver.brock@tu-berlin.de)

### A framework to perceive the world through interactions

OMIP creates a interconnected group of recursive loops that cooperate to quickly perceive robot's environment using knowledge about interactions and how they affect the sensor signals.
The framework is based on the work presented [here](http://www.robotics.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/martinmartin_ip_iros_2014.pdf) and [here](http://www.redaktion.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/martin_hoefer_15_iros_sr_opt.pdf).

## Quickstart
Assuming you already have installed ROS and have created a catkin environment.
- Check out OMIP and omip_msgs from the repository in your catkin environment:<br/>
```git clone https://gitlab.tubit.tu-berlin.de/roberto.martinmartin/omip.git```
```git clone https://gitlab.tubit.tu-berlin.de/roberto.martinmartin/omip_msgs.git```
- Install dependencies:<br/>
 - Boost, Eigen library.
 - Terminator (special terminal)
 - ROS packages OpenCV, PCL, openni, openni2, cmake-modules, BFL:<br/>
```sudo apt-get install ros-indigo-pcl-ros ros-indigo-openni-launch ros-indigo-openni-camera
ros-indigo-openni2-launch ros-indigo-openni2-camera ros-indigo-cmake-modules ros-indigo-bfl```
 - Copy omip/thirdparty/bflConfig.cmake into your_ros_install_dir/share/bfl.
 - RVIZ plugin to compute the area of the image occluded by the robot (you will need a URDF model of your robot!):<br/>
```git clone https://github.com/roberto-martinmartin/rviz_plugin_camerarenderpublisher.git```
 - Libpointmatcher (optional, to run the newest version of the shape tracker). Follow the instructions [here](https://github.com/ethz-asl/libpointmatcher/blob/master/doc/Compilation.md) 
- Download one of the the demo rosbags from [here](https://owncloud.tu-berlin.de/index.php/s/uDSTdI3FDQagfL1) (several GB!) or start an openni node.
- Start a roscore
- Run OMIP. For example:<br/>
```rosrun omip_launch omip.sh --omip=1 --rgbd=0```
- Play the demo bag or use the RGB-D stream provided by the openni node.

You can launch OMIP with 3 different system architectures:
1. **Main architecture**. Three estimation levels: feature tracking, feature-based rigid body tracking, kinematic model estimation. This option can execute only using RGB-D images and therefore requires less computational power. 
To try this option, download one of the rosbags with the "_imgs" suffix and launch OMIP using the option "--omip=1" (or "--omip=2" if you want that the terminals 
remain open after finishing the execution, for debugging purposes).
2. Four estimation levels: feature tracking, feature-based rigid body tracking, kinematic model estimation, shape reconstruction. This option requires point clouds and is therefore computationally more expensive. 
To try this option, download one of the rosbags with the "_full" suffix and launch OMIP using the option "--omip=3".
3. Five estimation levels: feature tracking, feature-based rigid body tracking, kinematic model estimation, shape reconstruction, shape-based rigid body tracking. This option requires also point clouds and is therefore computationally more expensive. 
To try this option, download one of the rosbags with the "_full" suffix and launch OMIP using the option "--omip=4".

For more information on how to use OMIP and activate/deactivate modules, checkout out the documentation, the wiki pages or run:<br/>
```rosrun omip_launch omip.sh --help```

More frequent problem: The feature tracking is not running and looks like frozen -> Check in feature_tracker/cfg/feature_tracker_cfg.yaml the depth_img_topic name. Depending if you are using
openni or openni2 (or rosbags generated from one or the other package) the name of the topic for the depth maps is different.

### What is OMIP?

Online Multimodal Interactive Perception (OMIP) is a framework to exploit the interaction capabilites of a robot to reveal and perceive its environment. 
The information to perceive this environment is contained in the high dimensional, continuous, multimodal sensor stream of the robot.
Our framework offers a way to interpret this stream based on prior knowledge encoded into recursive estimation loops. 
The prior knowledge used for perception includes physics laws (rigid body physics, kinematics, ...) and knowledge about the correlation between robot actions and responses of the environment.

Based on the recursive estimation schema - prediction/correction -  our framework can cope with the high amount of data provided by the robot's sensors **in an online manner**.
The key (and the main idea of our framework) is to factorize the perceptual problem into smaller *perceptual units*, solve them with single recursive estimation loops and connect all the loops tightly. 
The connection of the loops defines a bidirectional information flow between loops: a bottom-up flow to pass estimations as measurements to more abstract levels, and a top-down flow to pass predicted measurements as predicted next states to less abstract levels.
By connecting the loops our framework can interpret the combined sensor-action stream as evidence of concepts of different level of abstraction.

In the given demo, an RGB-D stream of an interaction is interpreted simultaneously as a moving visual field (motion of features and surfaces), multiple moving rigid bodies and changes in the kinematic state of an articulated object.

## Structure

OMIP is designed to work in ROS (Robot Operating System). 
ROS provides a suitable message-passing architecture to interconnect recursive estimation loops and propagate their information (estimations and predictions).

Each recursive estimation level is realized as a ROS node that implements the interface *RecursiveEstimatorNodeInterface*. 
A recursive estimation level is defined by:
- Measurement
- State
- Priors:
 - About the relation between measurements and state (measurement model)
 - About the relation between the state at time t and t+1 (forward model)
 - About the uncertainty (of measurements and predicted states)

An iteration of our recursive estimation levels begin when a new measurement arrives (execute-on-demand). This behavior is encoded using callbacks.
An iteration of a recursive estimation level implies correcting the state, predicting the next state and publishing the results. 
Note that we invert the classical order predict-correct; this allows us to make us of the predictions already in an earlier point in time! 
This *trick* is crucial in our system: the predictions are passed to other levels and can be used as alternative forward models based on extra priors.

Internally, each level contains at least one recursive estimation filter. These filters implement the interface *RecursiveEstimatorFilterInterface*.
The levels call the corresponding correct-predict methods of the filters and pass the measurements/states up and down.

You can extend the OMIP framework with new recursive loops. To do that, create a filter that implements the filter interface. Then create a node that implements the node interface and contains an instance of the filter interface. Connect your new node to existing nodes and sensors.