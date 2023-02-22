---
icon: device-camera-video
order: 100
---
# Replaying Recorded Data
During software development it is common to test software and algorithms on pre-recorded real-world and simulated data.  ROS enables this ability through the use of `.bag` files.  Each `.bag` contains the topics which were selected to be recorded at the time the file was created.
!!!info Creating Rosbags
See the [ROS Wiki article](http://wiki.ros.org/rosbag/Commandline#record) on the rosbag tool for more information on creating, playing, and inspecting `.bag` files.
!!!

# ROS Dataset Locations
The AVC team typically stores recorded datasets in on the NAS, which is mounted to most AVC servers and Docker environments under `/mnt`.  External datasets (such as BDD100K, NuScenes, etc.) are mounted under `/mnt/datasets-external` while internal datasets, produced by the SOAR team, are available under `/mnt/datasets` and `/mnt/dataset-dropbox`.

# Playing ROS Bag Data
To play the ROS bag data, a system with ROS installed must be used. The most convenient way to do this for most will be through the use of the [AVC vehicle docker](https://gitlab.msu.edu/canvas/soar/software/docker). Once inside the docker, simply start `roscore`, then run:

```bash
rosbag play --clock path-to-desired.bag
```  
This will play the rosbag, allowing the system to act as if it is ingesting real time data.

If a large number of bag files are included in the folder (i.e. `bag_1.bag, bag_2.bag, ... bag_n.bag`)and you would like to run over all of them, use:

```bash
rosbag play --clock bag_*.bag
```

Finally, to loop continuously, as is often desirable when testing, use the `-l` flag:

```bash
rosbag play --clock -l bag_*.bag
```

# Visualizing Using Rviz
!!!warning Using Simulated Time
To visualize pre-recorded `.bag` data using Rviz, first set the directive to use simulated time (i.e. the time the data was recorded, and not the current time), otherwise Rviz will see the data is stale and ignore it.

```bash
rosparam set /use-sim-time true
```
!!!

First, start up `rviz` to graphically display data.

```bash
rosrun rviz rviz
```

!!!warning Transforms
To properly display data, the transforms for the vehicle must be published. You can verify this by checking if the `/tf_static`
topic is running by listing active topics using the command:
```bash
rostopic list
```
If the /tf_static topic is not published, then you can launch the transform manager with this command:
```bash
roslaunch ~/vehicle/src/bolt_launch/launch/tf_tree.launch
```
!!!

Once the `/tf_static` topic is published (through either of the methods above), the fixed frame may be selected. To do so, select the the drop-down in the upper left corner of the window under "Displays > Global Options > Fixed Frame" and select "base_link".  

Next, below the "Displays" window should be a button which says "Add", select this, then select the "By Topic" tab – if the ROS bag is playing, topics should be listed in this tab.  Lidar objects will be viewable as "PointCloud2" objects, and cameras as "Images" and so on.

!!!info Visualizing Points
One final note, with lidar objects, it is helpful to visualize these as "Points" as opposed to "flat squares" as is the default option. Not only are points easier to see, but they are also typically faster to render on weaker systems. Also consider setting Alpha to 1 on lower-end systems to reduce render times.
!!!