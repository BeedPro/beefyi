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

## Configure CMakeLists.txt

For custom messages, services, actions please refer to the `CMakeLists.txt` that is automatically generated with `catkin_create_pkg`.

Quickish places to add:

```cmake
find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation)

add_<messages/services/actions>_files(
  FILES
  <file.srv>
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
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

## ROS Services

Services are synchronous (request/response) communication between nodes.

### Create a Service Definition

Example File: `srv/AddTwoInts.srv`

```srv
int64 a
int64 b
---
int64 sum
```

Place inside:
`<pkg>/srv/<file.srv>`

### Service Server Example

```python
#!/usr/bin/env python

from your_package.srv import AddTwoInts, AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    rospy.loginfo(f"Adding {req.a} + {req.b}")
    return AddTwoIntsResponse(req.a + req.b)

rospy.init_node('add_two_ints_server')
service = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
rospy.loginfo("Ready to add two ints.")
rospy.spin()
```

### Service Client Server

```python
#!/usr/bin/env python

import rospy
from your_package.srv import AddTwoInts

rospy.init_node('add_two_ints_client')
rospy.wait_for_service('add_two_ints')

try:
    add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
    resp = add_two_ints(3, 5)
    rospy.loginfo(f"Sum = {resp.sum}")
except rospy.ServiceException as e:
    rospy.logerr(f"Service call failed: {e}")
```

### Command-line Tools

```bash
rosservice list
rosservice info /add_two_ints
rosservice call /add_two_ints "a: 2 b: 3"
rossrv show your_package/AddTwoInts
```

## ROS Actions

Actions are asynchronous, providing feedback and allowing preemption.

### Create an Action Definition

Example File: `action/Fibonacci.action`

```action
int32 order
---
int32[] sequence
---
int32[] feedback_sequence
```

Place inside:
`<pkg>/action/<file.action>`

### 💻 **3. Python Action Server**

```python
#!/usr/bin/env python

import rospy
import actionlib
from your_package.msg import FibonacciAction, FibonacciFeedback, FibonacciResult

class FibonacciActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('fibonacci', FibonacciAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        feedback = FibonacciFeedback()
        result = FibonacciResult()

        feedback.sequence = [0, 1]

        for i in range(2, goal.order):
            if self.server.is_preempt_requested():
                rospy.loginfo("Goal preempted")
                self.server.set_preempted()
                return

            feedback.sequence.append(feedback.sequence[-1] + feedback.sequence[-2])
            self.server.publish_feedback(feedback)
            rospy.sleep(1.0)

        result.sequence = feedback.sequence
        self.server.set_succeeded(result)

rospy.init_node('fibonacci_server')
server = FibonacciActionServer()
rospy.spin()
```

---

### Action Client

```python
#!/usr/bin/env python

import rospy
import actionlib
from your_package.msg import FibonacciAction, FibonacciGoal

def feedback_cb(feedback):
    rospy.loginfo(f"Feedback: {feedback.sequence}")

rospy.init_node('fibonacci_client')
client = actionlib.SimpleActionClient('fibonacci', FibonacciAction)
client.wait_for_server()

goal = FibonacciGoal(order=5)
client.send_goal(goal, feedback_cb=feedback_cb)
client.wait_for_result()

result = client.get_result()
rospy.loginfo(f"Result: {result.sequence}")
```

### Command-line Tools

```bash
rosaction list
rosaction info fibonacci
rosaction show your_package/Fibonacci

# Using the action client tool (rqt or actionlib_tools)
rostopic pub /fibonacci/goal your_package/FibonacciActionGoal "{goal: {order: 5}}"
rostopic echo /fibonacci/feedback
rostopic echo /fibonacci/result
```

## Service vs Action Summary

| Feature             | Service                | Action                      |
| ------------------- | ---------------------- | --------------------------- |
| Communication Style | Synchronous (blocking) | Asynchronous (non-blocking) |
| Use Case            | Quick request/response | Long-running task           |
| Feedback            | None                   | Periodic feedback available |
| Preemption          | Not possible           | Possible                    |
| Tools               | `rosservice`           | `actionlib`, `rostopic`     |

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
