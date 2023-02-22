---
icon: rocket
order: 100
---
# Getting Started
!!!warning Before You Read
If you haven't already, we'd recommend that you check out the [ROS Tutorial 
Project](https://gitlab.msu.edu/canvas/soar/public/ros-tutorial-project). This is
a tutorial that will allow you to run through all the development basics on a test repo. 
It also has some Linux tips that you will find useful if you are unfamiliar.
!!!

## Cloning the Vehicle Repo
The [vehicle repo](https://gitlab.msu.edu/canvas/soar/software/vehicle) is where
all of the vehicle software is stored. You can clone this directory anywhere, but 
I'd recommend that you create a separate directory, say `soar/`, in order to 
separate all your vehicle files. In this new directory, run:
```
$> git clone --recurse-submodules https://gitlab.msu.edu/canvas/soar/software/vehicle.git
```
And that's it for vehicle code.

## Spinning up the Docker
For development purposes, it is ideal to keep user environments uniform so that
code works everywhere, not just on whatever system you happen to be on. For this,
we use Docker, which is a [container](https://www.docker.com/resources/what-container/)
that standardizes our OS and dependencies across systems. When you run our Docker 
container, you are running a complete copy of the vehicle system, just without 
any sensors or vehicle hardware attached.

Our [docker repository](https://gitlab.msu.edu/canvas/soar/software/docker) is 
where you will find the source. You should have a workable linux environment if 
you followed the ROS tutorial project, so spin that up. Firstly, cd into the same
directory where you stored `vehicle/` and clone the docker repo on commandline with:
```
$> git clone --recurse-submodules https://gitlab.msu.edu/canvas/soar/software/docker.git
```
Then cd into the `docker/` directory and run:
```
$> sh build.sh
```
This will build the docker image. Don't be alarmed if this takes a while, you are
building an entire OS from scratch. 
To run the docker image, run:
```
$> sh run.sh
```
And now your docker image is running!

## Building the Repository
Finally, you can build the vehicle code. As our vehicle codebase is a mix of C++
and Python, it does need to be compiled and built before running. Run
```
$> cd vehicle/
$> catkin_build
```

For next steps, see *Branching*