---
icon: file-directory
order: 98
---
# Package Creation
!!!warning Before You Begin
Make sure that you have read/followed the instructions for [configuring your environment](~/workflow/getting_started) and [creating a branch](~/workflow/branching_and_merging) before creating a package. This will ensure that you're creating it in the right place!
!!!


## Command-line Creation
First up is the creation of a package using command-line tools.
You should begin by getting yourself into the correct directory under `/vehicle/src/` corresponding to
the purpose of your node. If you are developing a computer vision node using the cameras on the vehicle, you
would want the command to be `cd vehicle/src/perception/camera/`. If you have any trouble
figuring out the right spot, ask!

The next step is to actually create the package needed for use. The command for this is:
```bash
catkin_create_package package_name package_dependency_1 ... package_dependency_n
```
Make sure to remove the curly braces, they are just indicating that you need actual
content there before running the command. `{package_name}` should be short, but descriptive.
It also should be all lowercase, with `_` as spaces. Package dependencies are just what libraries 
that your node will depend on, and here are a few examples:
 - `roscpp` and/or `rospy` for C++/Python nodes respectively. If you are not sure
    which language you are going to be using, feel free to include both as
    separate dependencies
   
 - `std_msgs`: A set of simple, standard message types.
 
 - `sensor_msgs`: A set of messages for common sensor types. If your node takes
   in raw sensor data, you will likely need this package
 
 - `geometry_msgs`: A set of messages containing geometric primitives. These are
   very commonly used by most packages, especially if they use transform data

## Package Contents
ROS packages are more than just folders. Inside the package filesystem there are multiple
different files needed to build, run, and keep track of everything going on in a ROS node.
Both package.xml and CMakeLists.txt won't necessarily need
editing unless you are adding ROS dependencies or developing with C++, but there are references below.

### package.xml
[ROS Reference for package.xml](http://wiki.ros.org/catkin/package.xml)

This is used to denote:
- Description: What the package is used for
- Maintainer: Who actually is developing for this package
- License: Who can use this code
These will be filled in with default values, and **you should change them on
initial package creation.**

Additionally, this will also contain the dependencies that you listed when creating
the package. If you need to add more later, you can use:
```
<depend>{new_dependency}</depend>
```
to easily add another dependency there.

### CMakeLists.txt
[ROS Reference for CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt)

This file is very important for the actual building of vehicle code. If you are
using C++, editing this file will be necessary when developing. The following section is modified from the [ROS Tutorial Project](https://gitlab.msu.edu/canvas/soar/public/ros-tutorial-project/):

---
First off, if you are going to be programming in C++, you will want to enable
C++11 features. There is a line early on in the file that reads:

`# add_compile_options(-std=c++11)`

Uncomment the line by removing the `#` symbol.

Next, note the lines that read:

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
```

This is another location for ROS dependencies that you'll need to add on to
if you add additional ROS dependencies later. Note that you need to edit both
`package.xml` AND `CMakeLists.txt` or else your package will not compile after
adding a dependency in one or the other, but not both.

A few lines later, you'll notice another call to `find_package()`:

```
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
```

If you use any non-ROS third party libraries for C++ code, you will add a call
to `find_package()` for each of these packages here. The most common packages
you might find here are OpenCV and LibPCL.

---

In the next block down, you will see a section titled
"Declare ROS messages, services and actions". This section is used if your
package *provides* any messages or services. If you are creating a custom message 
type in this package, specify that here.

---

!!!warning C++ Only
The next section, "build", is very important for C++ projects. If you are
only programming in Python, you can skip this section.
!!!

First off, there
is a call to `include_directories()` that we will want to modify. There are two
major things to add here:

1. Any non-ROS system libraries. Note that these will likely be CMake variables;
for instance, LibPCL uses the `${PCL_INCLUDE_DIRS}` as the include macro for
this section. This isn't required for this tutorial project, but will likely be
necessary for more complex projects.

2. The local `include/` directory. By convention, C++ header files should go
into the `tutorial_pkg/include/` directory, with further subdirectories 
preferably added to aid organization. This directory is created by default,
but you may notice here in the `CMakeLists.txt` file that it's been commented
out. You should uncomment it here.

Beyond the call to `include_directories()` there are several CMake commands
that are used.
The first, `add_library()`, is used when you want to compile a set of C++
code as a library and not as a full node, either for just your package or as
a library that can be used outside your package. We aren't doing that here, so
skip the `add_library()` command. Note the call to `add_dependencies()` right
afterward; we will be calling this ourselves, but not at this point in the file.

The next three commands are what we'll be using when compiling our code. The
`add_executable()` command allows you to set the output compiled node name
(the first argument), and then takes in the path to all the source files you
need to compile your node. You should uncomment this line, but depending on
how you structure your source code you may need to come back and add the 
paths to your source files later.

!!!danger Naming Conventions for Executables
**Always make your node have a `${PROJECT_NAME}_` prefix to the
output executable name!** Otherwise, if you name your executable the same as one
in another ROS package, it will fail to compile! You can remove the prefix using
the `set_target_properties()` command below the `add_executable()` command.
!!!

Moving on to the `add_dependencies()` command, unless you have dependencies on
system libraries (added with the `find_package()` command) or you compiled any
libraries for your project, you can uncomment this line and leave it as-is.