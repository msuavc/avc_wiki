---
icon: hubot
order: 100
---

# ROS Basics

For the vehicle to run as we need it to, it needs to have a backend program that
is able to coordinate the flow of data between the sensors and the software that
converts that sensor data into controls, detections, and other more relevant data
for the vehicle to run autonomously. ROS is the solution that the team, and many
others in the robotics industry, uses in order to do this coordination.

This page provides a general high-level overview of basic components in ROS, as
well as the command-line tools that allow for inspection of these components.

## What is ROS?

ROS is middleware used to lighten the development load of robotics applications.
Before software like ROS, developing any robotics platform required many man hours
of development on features needed to just get the system running. If you were
developing a system as complex as ours from scratch, you would need to develop
every component in the system, from data management to testing. The project
itself may even stall before the interesting research happens!

With ROS, most of the backend work is taken care of by the software itself. ROS
is also a modular framework, meaning that you can swap different components as
you see fit, only having to worry about inputs and outputs with each software
component and not about how to handle all the low-level dataflow and implementation
details.

## Relevant Links

More comprehensive introductions to ROS can be found here:

- [What Is ROS?](https://roboticsbackend.com/what-is-ros/)
- [Introduction to ROS](http://wiki.ros.org/ROS/Introduction)

Additionally, if at any time you cannot find the answers you are looking for
here, the ROS wiki is a great resource that will fill you in on the finer
details without having you frantically google everything:

- [http://wiki.ros.org/ROS](http://wiki.ros.org/ROS)

Finally, each section will have links directing you towards comprehensive
documentation provided by the ROS development team itself. Most of the information
is explained in much more detail there, and many of the command-line instructions
found below are a simple subset of the functionality they actually offer.
**It is encouraged that you read this links during/after your time reading this document.**

## Packages

[ROS Reference for Packages](http://wiki.ros.org/Packages)

Packages are at the core of ROS’ system. Packages contain all nodes, services, message types, and more that are present in the system. At a basic level, however, they are just directories containing source code and configuration files allowing for separation of codebases based on their purpose. This is ROS’ way of handling part of directory management for you.

### Directories and Files

A ROS package, at a basic level, is just a parent directory. This parent can contain anything, but the files subdirectories that are relevant to the package are explained below:

#### `./src/`

This is where all of your python source code will be stored, and in that source code will be nodes, services, and other coded functionality on the vehicle.

#### `./src/package_name`

This is where all of your C++ source code will be stored, and in that source code will be nodes, services, and other coded functionality on the vehicle.

#### `./CMakeLists.txt`

This is the cmake build file for the package, and it is required for building and running the package. Stored in here are all the cmake directives and other information needed to actually build the package for use. I will not go into the contents of this file here, as it is quite complex and is created when building a package (_See Development in ROS)._ For more information about the contents of this file, you can also refer to [ROS’ CMakeLists documentation](http://wiki.ros.org/catkin/CMakeLists.txt).

#### `./package.xml`

This file is the package manifest, and it is another file required for building and running from a package. Information here is similar to `CMakeLists.txt` , and similarly, I will not go into the file contents due to complexity. For information, you can also refer to [ROS’ package.xml documentation](http://wiki.ros.org/catkin/package.xml).

#### `./msg/`

This folder will contain all custom message types defined in the package. Most packages will not need this, as there are both ROS provided and custom defined messages already defined in the system that should suffice. If not, however, this folder is here.

_More about ROS messages below._

#### `./srv/`

This folder will contain all custom defined service types. If you are creating a service, you will define the data the service will request and reply here.

#### `./scripts/`

This folder will contain any executable scripts needed to run the package here, as well as any you develop that will be relevant to package use. Nodes/services will not be defined here, but if you, for example, use a shell script in a node or service, it would be placed here.

#### `./include/package_name`

This directory is where all of your c++ header files will go if applicable.

### Package Inspection

ROS provides the `rospack` command-line tool in order to inspect packages:<https://docs.ros.org/en/independent/api/rospkg/html/rospack.html>

```bash
# this will list all packages
rospack list

# this will display the absolute path of the package
rospack find {package_name}

# this will display the dependencies of a given package
rospack depends {package_name}
```

## Data Flow

Data flow in ROS is described more technically here: [ROS Concepts](http://wiki.ros.org/ROS/Concepts)

In ROS, data flow is handled by a peer-to-peer network of individual pieces of software. These "nodes" are able to request and output data as you see fit. ROS handles this quite smartly, and loosely couples building blocks of software and data together to allow for software to be changed in a modular fashion.

### Messages

[ROS Reference for Messages](http://wiki.ros.org/msg)

Messages, and message types, are the primary abstraction used for data in the system. ROS provides wrappers for many primitive types to enable cohesion in the system.

Each message in ROS has a type, and ROS provides many types out of the box. These types can be easily googled, but a full list of primitive types supplied is found here:

[http://docs.ros.org/en/noetic/api/std_msgs/html/index-msg.html](http://docs.ros.org/en/noetic/api/std_msgs/html/index-msg.html)

For example, the ROS type for a header, which provides data for the order of messages being sent and the sequence in which they are sent is represented as so:

```bash
# Standard metadata for higher-level stamped data types.
# NOTE: stamped types mean that they have this header
# This is generally used to communicate timestamped data
# in a particular coordinate frame.
#
# sequence ID: consecutively increasing ID
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
```

Googling these types will only get you so far, as in ROS there is also the ability to define custom types. These types will not be documented anywhere besides in the codebase, and there is a command-line tool for accessing the specifications for each type: [http://wiki.ros.org/rosmsg](http://wiki.ros.org/rosmsg). Below are some basic commands that will be useful:

```bash
# this is the main command you will use to inspect message types
rosmsg show {message_type}

# for example, to output what is displayed above, you would run
rosmsg show std_msgs/Header
```

### Topics

[ROS Reference for Topics](http://wiki.ros.org/Topics)

Topics are a fundamental way ROS moves messages through the vehicle system. Topics are published and subscribed to by nodes, allowing nodes to exchange data with each other.

Topics have a very specific naming scheme in ROS:

```bash
# this is an example of a topic in ROS
# "perception" and "camera" define the specific namespace that the topic
# resides in
# "image" describes the actual data that the nodes are providing in that
# namespace
/perception/camera/image
```

All messages passed via nodes can be accessed through these topic names, and all data published in ROS will have an associate topic. Topics can be accessed by any other node in the system, as long as the nodes providing them are running.

To access topics, and see their content, a command-line tool called `rostopic` is utilized. Below are some basic commands using that tool: [http://wiki.ros.org/rostopic](http://wiki.ros.org/rostopic)

```bash
# this will produce a list of topics currently available in the system
rostopic list

# this will show you more information about a specific topic, like the message
# type, which node is providing it, etc.
rostopic info {topic_name}

# this will show you the actual data in the topic
rostopic show {topic_name}
```

### Nodes

[ROS Reference for Nodes](http://wiki.ros.org/Nodes)

A node in ROS is one of the fundamental building blocks of the software. A node serves as the basic computational unit in ROS, and the brunt of the work you do on the vehicle is node-based.

A node in ROS can both publish and subscribe to topics in the ROS system. Nodes can also provide services, which are discussed in the next section.

> Publishing: Providing data to the ROS system
> Subscribing: Getting data from another node in the ROS system

Each node has a name in the system that is user defined, and these are part of what determine namespaces in ROS. A node handling raw camera input and output could be named `/camera`

In a ROS node, data is manipulated via the programming languages C++ and Python. Each node is responsible for a step in the process of data movement in the system. For example, to make the car detect objects, there [http://wiki.ros.org/rosnode](http://wiki.ros.org/rosnode)will be a detection pipeline needed starting from the camera and ending with whatever handles responding to detections. This pipeline would have multiple nodes, starting with `/camera` publishing to the topic `/camera/image` , and that would be picked up by the node `/detector` which would then publish its detections on `/detector/detections`. These detections would then be used by other nodes in the system as they see fit.

Similar to messages and topics, there is a command-line tool to inspect nodes: [http://wiki.ros.org/rosnode](http://wiki.ros.org/rosnode). Some basic commands for inspecting nodes are found below:

```bash
# this will list all the nodes available in the current system
rosnode list

# this will display information about a node, such as what messages are being
# published by and subscribed to by the node
rosnode info /{node_name}

# this will kill a node that is running
rosnode kill /{node_name}
```

### Services

[ROS Reference for Services](http://wiki.ros.org/Services)

With publishing and subscribing, you have a data flow that provides data from many nodes in the system to many other nodes in the system, The caveat is that dataflow in this structure is largely one way.

ROS services provide a different way of implementing data flow in ROS, a request/reply based framework. This is very useful when building up quick computations and actions that do not need to be integrated in the larger system. For example, if you want a quick way to change settings on the vehicle, use a service! You can configure a service to change those settings, and then configure it to reply on a success.

ROS services are defined and displayed similar to messages. The key difference is that a message will have one single section of fields defining the datatypes shown in the message, whereas services have two. The first is the `request` body, and this defines all the data the service needs to handle the request. The second is the `response` body, which defines the data the service will provide upon completing the request. A service will look something like this (example pulled from [http://wiki.ros.org/srv](http://wiki.ros.org/srv)):

```bash
#request constants
int8 FOO=1
int8 BAR=2
#request fields
int8 foobar
another_pkg/AnotherMessage msg
--- # delineater defining where request types end and response types begin
#response constants
uint32 SECRET=123456
#response fields
another_pkg/YetAnotherMessage val
CustomMessageDefinedInThisPackage value
uint32 an_integer
```

To see ROS services from a command-line interface, you can use two tool: `rossrv` , which functions very similarly to `rosmsg`, and `rosservice` , which is used to actively request services themselves and display services actively running in the system. Here are some examples for `rossrv` and `rosservice`:

```bash
# rossrv
# this will display output similar to the above codeblock
rossrv show {service}

# this will list all services (even offline services)
rossrv list

# this will list all services a package provides
rossrv package

# this will list all packages that contain a service
rossrv packages
```

```bash
# rosservice
# this will print info about an active service
rosservice info {service}

# this will list all active services
rosservice list

# this will display the type for the service
rosservice type {service}

# this will call a service will specific args
rosservice call /{service-name} {service_input}
```

### Parameters

[ROS Reference for Parameters](http://wiki.ros.org/Parameter%20Server)

Parameters are how to set global constants in ROS. These are useful in many areas, such as configuration settings, numerical constants, and much more. You will use services for constants designed not to be changed often, as parameters are not used for high frequency dataflow.

As these are designed to be constants, the datatypes available for params are as follows:

- 32-bit integers
- booleans
- strings
- doubles
- iso8601 dates
- lists
- base64-encoded binary data

The command-line tool `rosparam` displays relevant information about parameters and their contents:

```bash
# this lists parameter names
rosparam list
rosparam list /{namespace}

# this gets a parameter value
rosparam get {parameter_name}

# this sets a parameter valie
rosparam set {parameter_name} {value}
```

### Bags

[ROS Reference for Bags](http://wiki.ros.org/rosbag)

In a robotics system, you may not have access to the robot whenever you need to develop something. Additionally, it is not practical to assume that you can spin up the vehicle to actively test what your software does in response to vehicle data. ROS provides a data storage solution called `rosbags` that solves this.

Specifically in our development environment, you have access to bags recorded at many times of the year with sensor data from the vehicle. These bags can be played, and when they are played, will spawn topics with data collected from that time. This will simulate what the car would actually output when it is running, and will allow you to rapidly test and develop.

Similar to all the other items above, there is a command-line tool for you to create and play these bags: [http://wiki.ros.org/rosbag/Commandline](http://wiki.ros.org/rosbag/Commandline)

```bash
# this will record a bag of topics specified
rosbag record {topic1} {topic2}

# this will display the start/end times of bags, and the topics contained
rosbag info {bag_file}

# this will play a bag
rosbag play {bag_file}

# this will loop a bag/bags
rosbag play -l {bag_file}
```
