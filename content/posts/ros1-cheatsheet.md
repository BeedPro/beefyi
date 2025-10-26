---
title: ROS1 Cheatsheet
description: A space for updates, reflections, and snippets from beed. This is beefyi — Beed For Your Info.
date: 2025-10-23
tags: ["cheatsheet", "draft"]
---

# ROS Noetic

**ROS Noetic** (Robot Operating System) is the final and most stable release of ROS 1, a framework for building and connecting robot software. It’s designed for Ubuntu 20.04 with Python 3 support and comes packed with tools, libraries, and community packages. However as of writing it has gone out of support from the ROS community and you should start using/migrating to ROS2. All code samples will be written in Python.

**Useful resources:**
• [ROS Wiki](http://wiki.ros.org)
• [ROS Documentation](https://docs.ros.org)
• [ROS Answers (Community Q&A)](https://answers.ros.org)
• [ROS GitHub](https://github.com/ros)

## Using apptainer

As I'm learning ROS, it was recommended to use containerisation software to run all the ROS tools in an isolated environment. For my setup, I'll be using [Apptainer](https://apptainer.org/) to manage and run everything inside a container:

```
apptainer run -B /run/user/$UID /<path_to_container.sif>
```

Anything in wrapped with "<>" are templates and should not be copied into the command but replaced accordingly to the context the command. For example in the above command "\<path_to_container\>" should be replaced with the actual path of the `.sif` file.

Assume that any command shown in this post is run within this container, code can be written in any IDE of your choice but the commands need to be run in the container.

## ROS Environment Setup

In ROS, a workspace (ws) is a directory where you build, modify, and organise your ROS packages and projects. We will be running our custom nodes and everything in the packages here.

```bash
# Create and source your workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Package Creation

**Packages** are the core building blocks that contain your code, nodes, launch files, and configuration, essentially everything needed for a specific function or feature in a robot system.

```bash
cd ~/catkin_ws/src
catkin_create_pkg <pkg_name> rospy <dep1> <...> <depn>
```

Where `<dep1> <...> <depn>` is all the packages needed for the package. Note I leave rospy there since we will be using Python and do not worry you can go back to this and add more dependencies in the `packages.xml` file that is created after running this command.

### Full Package structure

```
my_package/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── my_node.py
└── scripts/
    └── talker.py
```

## Writing a Python Node

**Example: `talker.py`**

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        msg = "Hello ROS Noetic! %s" % rospy.get_time()
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

### Subscribing to Topics

**Example: `listener.py`**

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```

### Running Nodes

```bash
# Make executable
chmod +x scripts/<name>.py

# Build the workspace
cd ~/catkin_ws
catkin_make

# Source the workspace
source devel/setup.bash

# Run roscore
roscore

# In a new terminal
rosrun <pkg> <name>.py
```

### Node Commands

```bash
# Check node info
rosnode info /talker

# List active nodes
rosnode list

# Kill a node
rosnode kill <node_name>

# Kill all nodes
rosnode kill -a
```

### Topic commands

```bash
# List topics
rostopic list

# Echo topic messages
rostopic echo /chatter

# Info about a topic
rostopic info /chatter
```

## ROS Launch

**Example of a launch file:**

```xml
<launch>
	<include file="$(find <pkg>)/launch/<file.launch>" />
	<node pkg="<pkg>" type="<node>" name="<name>">
		<remap from="<from_topic>" to="<to_topic>" />
	</node>
</launch>
```

**Run it**

```bash
roslaunch <pkg> <file.launch>
```

## Rosbag

To record logs from a topic and then can replay it for debugging purposes

```bash
rosbag record /<topic>
```

```bash
rosbag play <file.bag>
```

### ROS Plot

We can use `rqt_plot` to visualise the messages of the topic:

```bash
rosrun rqt_plot rqt_plot
```

Use `rosmsg` to see what lines you can add to the plot and visualise.

```bash
rosmsg info /<topic>
```

## ROS Actions

TODO: Complete

## ROS Services

TODO: Complete

## Useful ROS Commands

```bash
# Running Stage simulator Rviz
rosrun rviz rviz -d `rospack find rosplan_stage_demo`/config/rosplan_stage_demo.rviz &

# Running Stage Simulator
roslaunch rosplan_stage_demo empty_stage_single_robot.launch &
```

## Common Message Types

| Type                  | Import                                | Example Use         |
| --------------------- | ------------------------------------- | ------------------- |
| `std_msgs/String`     | `from std_msgs.msg import String`     | Chat or text data   |
| `geometry_msgs/Twist` | `from geometry_msgs.msg import Twist` | Velocity commands   |
| `sensor_msgs/Image`   | `from sensor_msgs.msg import Image`   | Camera data         |
| `nav_msgs/Odometry`   | `from nav_msgs.msg import Odometry`   | Robot position data |

## Workspace Maintenance

```bash
# Clean workspace
catkin_make clean

# Rebuild everything
catkin_make -DCMAKE_BUILD_TYPE=Release

# Re-source environment
source devel/setup.bash
```

## Debugging Tips

```bash
# Check log files
roscd log
less latest/rosout.log

# Print debug messages
rospy.logdebug("Debug message")
rospy.loginfo("Info message")
rospy.logwarn("Warning message")
rospy.logerr("Error message")
```
