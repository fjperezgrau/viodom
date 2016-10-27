# viodom [![Build Status](https://travis-ci.org/fjperezgrau/viodom.svg?branch=master)](https://travis-ci.org/fjperezgrau/viodom)
ROS package (indigo) for visual odometry. This package contains one single node: viodom_node, which estimates robot motion based on incoming raw images and IMU mesaurements from the Visual-Inertial (VI-) Sensor. To correctly estimate the motion, the node first needs to wait for a few seconds to initialize an IMU filter.
It depends on OpenCV (installed alongside with ROS desktop-full) and Eigen library which must be installed in your system:
```
sudo apt-get install libeigen3-dev
```

**Who do I talk to?**
* Fernando Caballero (fcaballero@us.es), University of Seville, Spain
* Francisco J Perez Grau (fjperezgrau@gmail.com), CATEC (Center for Advanced Aerospace Technologies), Spain

# Nodes
Currently this package only includes one node which takes in a stereo pair of images and IMU data, and outputs pose estimates. Images are rectified using calibration information. On startup, an IMU filter needs to be initialized so the robot needs to be quiet for a few seconds. Some parameters can be changed without having to restart the node, as indicated in the dynamic reconfiguration file in subfolder 'cfg'.

**Subscribed Topics**
* left_cam/image_raw (sensor_msgs/Image): Left camera raw image. 
* left_cam/camera_info (sensor_msgs/CameraInfo): Left camera calibration information. 
* right_cam/image_raw (sensor_msgs/Image): Right camera raw image. 
* right_cam/camera_info (sensor_msgs/CameraInfo): Right camera calibration information. 
* imu_topic (sensor_msgs/Imu): IMU topic. 

**Published Topics**
* viodom_transform (geometry_msgs/TransformStamped): Robot's estimated pose in the odom frame. 
* point_cloud (sensor_msgs/PointCloud2): 3D point cloud based on the input stereo pair. 
* tf (tf/tfMessage): Publishes the transform from odom to base_link. 

# Demo
The package includes a launch file with parameters ready to use with a dataset recorded at CATEC. This dataset includes images, IMU data and ground-truth localization taken from a VICON motion capture system.
The bagfile can be downloaded from https://drive.google.com/open?id=0B3msAWZJELBTYXg3OE0zWmd0eVU

# Citing
If you use viodom in an academic context, please cite the following publication:

http://ieeexplore.ieee.org/document/7502653/

```
@INPROCEEDINGS{7502653,
  author={F. J. Perez-Grau and F. R. Fabresse and F. Caballero and A. Viguria and A. Ollero},
  booktitle={2016 International Conference on Unmanned Aircraft Systems (ICUAS)},
  title={Long-term aerial robot localization based on visual odometry and radio-based ranging},
  year={2016},
  pages={608-614},
  doi={10.1109/ICUAS.2016.7502653},
  month={June},}
```

The presentation material can be found in https://drive.google.com/open?id=0B3msAWZJELBTbGJaUjBJOW11NTg

