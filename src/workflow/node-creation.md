---
icon: note
order: 97
---
# Node Creation
Now that you've [created a package](~/workflow/package_creation), you're ready to get into actual development. 
The first step to actual development is creating a node, and to go through that 
process, we will use a template node to illustrate how we structure our ROS code.

## First Steps
The first thing you should do before using anything in this doc is actually creating
a python file. You can do this on linux by executing
```
touch [node_name].py
```
where `[node_name]` is the name of your node. Then you can copy the template and use
it in your node's code.

## Template Node
### Code
```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


class base_node():
  def __init__(self):
    self.sub = rospy.Subscriber('/subscribed_topic', MessageType self.callback)
    self.pub = rospy.Publisher('/published_topic', MessageType, queue_size=2)

  def callback(self, msg):
    self.pub.publish(msg)

if __name__ == '__main__':
  try:
    rospy.init_node('base_node', anonymous=True) 
    base_node() 
    rospy.spin()
  except RuntimeError as e:
    pass
```
### Breakdown
As you can see, a minimum functioning node does not require a ton of code, but we
will still break it down into parts so your understanding is correct.
#### Shebang
```python
#!/usr/bin/env python3
```
First up, we have the shebang. This tells your underlying Linux OS how to 
interpret this file. As you are creating a python node, this will always be 
`#!/usr/bin/env python3`.
#### Imports
```python
import rospy
from std_msgs.msg import String
```
Next up is your imports. You can add any python packages present on the system here,
but there are a few important ones that you shouldn't miss. If you recall your 
reading of [Package Creation](~/workflow/package-creation.md),
you will remember that you declared some dependencies, such as `std_msgs`, `rospy`, and the likes.
These will need to be imported here to be used in your python script, and likewise they will need to be included in
your package dependencies. One important thing to note is that you should import
message types as needed rather than doing `import std_msgs.msg`.
#### Class
```python
class base_node():
  def __init__(self):
    self.sub = rospy.Subscriber('/subscribed_topic', MessageType, self.callback)
    self.pub = rospy.Publisher('/published_topic', MessageType, queue_size=2)

  def callback(self, msg):
    self.pub.publish(msg)
```
Now we get to the meat and bones of the operation, an object oriented ROS node.
There are two ROS specific parts to pay attention to here: the publisher/subscriber
initialization, and the callback function.
```python
def __init__(self):
    self.sub = rospy.Subscriber('/subscribed_topic', MessageType, self.callback)
    self.pub = rospy.Publisher('/published_topic', MessageType, queue_size=2)
```
First up is the initialization function. In here, we define the inputs and outputs of 
your rosnode in terms of rostopics. For your subscriber, there's a minimum of three arguments:
- `/subscribed_topic`
    - This is the topic name you want the node to receive data from
- `MessageType`
    - This is the message type of the topic you want to subscribe to. As covered in 
    the imports section, you will import message types from a `.msg` import, and then they
    will go here. If you are having trouble determining the message type of the topic you want,
    then run `rostopic info /topic_name`!
- `self.callback`
    - This will be the callback function that is run everytime a new message is received.
For your publisher, the arguments are:
- `/published_topic`
    - This is the topic name you want the node to publish data on. You choose this name
    yourself, so make it descriptive
- `MessageType`
    - This is the message type of the topic you want to publish. As covered in 
    the imports section, you will import message types from a `.msg` import, and then they
    will go here.
- `queue_size`
    - This determines how many messages will be queued before ros clears them from
    memory

A couple of notes:
- You can have multiple publishers and subscribers in one node, but there are some
things you will have to watch out for:
    - Each subscriber must have their own callback (If you want a single callback with synced messages,
    use [message_filters](http://wiki.ros.org/message_filters#Example_.28Python.29))
    - Each publisher should only be published once, and you need to use distinct topic 
    names for each publisher
- To publish a message of ANY type, you need to fill out the fields of each message manually.
Messages are just instances of a python class, and you will fill out each instance of a
message like you would member variables of a python class. For example, a [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)
is filled out like below (**these are not real values**):
```python
    pcl_message = PointCloud2()
    pcl_message.header.stamp = rospy.Time.now()
    pcl_message.header.frame_id = "world"
    pcl_message.height = 1
    pcl_message.width = 680
    pcl_message.fields = []
    pcl_message.is_bigendian = true
    pcl_message.point_step = 4
    pcl_message.row_step = 64
    pcl_message.data = points[1:64]
    pcl_message.is_dense = false
```

#### Main Function
Lastly, we have the main function:
```python
    if __name__ == '__main__':
      try:
        rospy.init_node('base_node', anonymous=True) 
        base_node() 
        rospy.spin()
      except RuntimeError as e:
        pass
```
This code block spins up the node and ensures that it is executing properly.
Some notable parts:
```python
if __name__ == '__main__':
```
This line is the actual function call for main() in python.
```python
rospy.init_node('base_node', anonymous=True)
```
This initializes the node with the name 'base_node'. Like every other name in this
template, it should be changed. This one should change to match the functionality
of the node.
```python
base_node() 
```
This initializes the node, and subsequently initializes the publishers and subscribers, meaning that the
node will begin to receive data.
```python
rospy.spin()
```
This puts the node into a cycle of waiting for the next message and then executing.
