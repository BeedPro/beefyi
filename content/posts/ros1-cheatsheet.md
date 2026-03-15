---
title: ROS1 Cheatsheet
description: A space for updates, reflections, and snippets from beed. This is beefyi — Beed For Your Info.
date: 2026-03-15
tags: ["cheatsheet"]
---

# ROS Noetic Cheatsheet

ROS Noetic is the final ROS 1 release and targets Ubuntu 20.04 with Python 3.
ROS 1 is still common in existing projects, but it is now end-of-life, so for
new systems you should plan a ROS 2 migration path.

This guide is a practical flow from setup to day-to-day debugging, with Python
examples.

**Useful links**

- [ROS Wiki](http://wiki.ros.org)
- [ROS Documentation](https://docs.ros.org)
- [ROS Answers](https://answers.ros.org)
- [ROS GitHub](https://github.com/ros)

## Running ROS in Apptainer

If you want a clean and reproducible environment, run your ROS image with
Apptainer:

```bash
apptainer run -B /run/user/$UID /<path_to_container.sif>
```

Anything inside `<>` is a placeholder. Replace it with your real value.

For the rest of this post, assume ROS commands are executed inside your
container shell.

## Workspace Setup

A ROS workspace is where your packages live and build.

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Tip: add `source ~/catkin_ws/devel/setup.bash` to `~/.bashrc` if this is your
main workspace.

## Create a Package

Packages are the unit of code sharing in ROS: nodes, launch files, messages,
configs, scripts.

```bash
cd ~/catkin_ws/src
catkin_create_pkg <pkg_name> rospy std_msgs
```

Add more dependencies later in `package.xml` and `CMakeLists.txt`.

Typical structure:

```text
my_package/
├── CMakeLists.txt
├── package.xml
├── launch/
├── msg/
├── srv/
├── action/
└── scripts/
    ├── talker.py
    └── listener.py
```

## First Publisher and Subscriber

`scripts/talker.py`

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def main():
    pub = rospy.Publisher("chatter", String, queue_size=10)
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = f"hello noetic @ {rospy.get_time():.2f}"
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
```

`scripts/listener.py`

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def callback(msg):
    rospy.loginfo("heard: %s", msg.data)


def main():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()


if __name__ == "__main__":
    main()
```

Build and run:

```bash
chmod +x scripts/talker.py scripts/listener.py
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash

roscore
# new terminal
rosrun <pkg_name> talker.py
# new terminal
rosrun <pkg_name> listener.py
```

## Daily CLI Commands

```bash
# nodes
rosnode list
rosnode info /talker
rosnode kill /talker

# topics
rostopic list
rostopic info /chatter
rostopic echo /chatter
rostopic hz /chatter
rostopic pub /chatter std_msgs/String "data: 'hi'" -1

# message introspection
rosmsg list
rosmsg show std_msgs/String
```

## Launch Files

`launch/demo.launch`

```xml
<launch>
  <node pkg="<pkg_name>" type="talker.py" name="talker" output="screen" />
  <node pkg="<pkg_name>" type="listener.py" name="listener" output="screen" />
</launch>
```

Run:

```bash
roslaunch <pkg_name> demo.launch
```

## Services

Use services for quick RPC-like interactions.

`srv/AddTwoInts.srv`

```srv
int64 a
int64 b
---
int64 sum
```

Server (`scripts/add_two_ints_server.py`):

```python
#!/usr/bin/env python3
import rospy
from your_package.srv import AddTwoInts, AddTwoIntsResponse


def handle(req):
    return AddTwoIntsResponse(req.a + req.b)


if __name__ == "__main__":
    rospy.init_node("add_two_ints_server")
    rospy.Service("add_two_ints", AddTwoInts, handle)
    rospy.loginfo("service ready")
    rospy.spin()
```

Client (`scripts/add_two_ints_client.py`):

```python
#!/usr/bin/env python3
import rospy
from your_package.srv import AddTwoInts


if __name__ == "__main__":
    rospy.init_node("add_two_ints_client")
    rospy.wait_for_service("add_two_ints")
    call = rospy.ServiceProxy("add_two_ints", AddTwoInts)
    resp = call(3, 5)
    rospy.loginfo("sum = %d", resp.sum)
```

CLI tools:

```bash
rosservice list
rosservice info /add_two_ints
rosservice call /add_two_ints "a: 2 b: 3"
rossrv show your_package/AddTwoInts
```

## Actions

Use actions for long-running jobs that need progress updates or cancel support.

`action/Fibonacci.action`

```action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

Server (`scripts/fibonacci_server.py`):

```python
#!/usr/bin/env python3
import rospy
import actionlib
from your_package.msg import FibonacciAction, FibonacciFeedback, FibonacciResult


class FibonacciServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "fibonacci", FibonacciAction, execute_cb=self.execute, auto_start=False
        )
        self.server.start()

    def execute(self, goal):
        feedback = FibonacciFeedback(partial_sequence=[0, 1])
        result = FibonacciResult()

        for _ in range(2, goal.order):
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                return
            feedback.partial_sequence.append(
                feedback.partial_sequence[-1] + feedback.partial_sequence[-2]
            )
            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)

        result.sequence = feedback.partial_sequence
        self.server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("fibonacci_server")
    FibonacciServer()
    rospy.spin()
```

Client (`scripts/fibonacci_client.py`):

```python
#!/usr/bin/env python3
import rospy
import actionlib
from your_package.msg import FibonacciAction, FibonacciGoal


def feedback_cb(msg):
    rospy.loginfo("feedback: %s", msg.partial_sequence)


if __name__ == "__main__":
    rospy.init_node("fibonacci_client")
    client = actionlib.SimpleActionClient("fibonacci", FibonacciAction)
    client.wait_for_server()
    client.send_goal(FibonacciGoal(order=8), feedback_cb=feedback_cb)
    client.wait_for_result()
    rospy.loginfo("result: %s", client.get_result().sequence)
```

Useful checks:

```bash
rostopic list | grep fibonacci
rostopic echo /fibonacci/feedback
rostopic echo /fibonacci/result
```

## State Machines with SMACH

When behavior grows beyond one callback, state machines make control flow explicit and debuggable.

Install tools:

```bash
sudo apt install ros-noetic-smach ros-noetic-smach-ros ros-noetic-smach-viewer
```

Minimal state machine with transitions:

```python
#!/usr/bin/env python3
import rospy
import smach
import smach_ros


class CheckBattery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["ok", "low"])

    def execute(self, userdata):
        rospy.sleep(0.5)
        battery_ok = True
        return "ok" if battery_ok else "low"


class Patrol(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata):
        rospy.loginfo("patrolling...")
        rospy.sleep(1.0)
        return "done"


class Dock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["charged"])

    def execute(self, userdata):
        rospy.loginfo("docking...")
        rospy.sleep(1.0)
        return "charged"


if __name__ == "__main__":
    rospy.init_node("smach_demo")
    sm = smach.StateMachine(outcomes=["finished"])

    with sm:
        smach.StateMachine.add("CHECK", CheckBattery(), transitions={"ok": "PATROL", "low": "DOCK"})
        smach.StateMachine.add("PATROL", Patrol(), transitions={"done": "finished"})
        smach.StateMachine.add("DOCK", Dock(), transitions={"charged": "finished"})

    sis = smach_ros.IntrospectionServer("smach_server", sm, "/SM_ROOT")
    sis.start()
    outcome = sm.execute()
    rospy.loginfo("state machine finished with: %s", outcome)
    rospy.spin()
    sis.stop()
```

Visualize live state transitions:

```bash
rosrun smach_viewer smach_viewer.py
```

SMACH rule of thumb:

- Use **service** for short blocking request/response
- Use **action** for cancellable long-running execution
- Use **SMACH** to orchestrate many steps (often calling services/actions
  inside states)

## TF2

TF2 tracks relationships between coordinate frames over time.

Quick commands:

```bash
# graph view
rosrun rqt_tf_tree rqt_tf_tree

# static PDF/graph output
rosrun tf2_tools view_frames.py

# numerical transform stream
rosrun tf tf_echo /map /base_link
```

Static transform example:

```bash
# x y z yaw pitch roll parent child
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link
```

Typical frame naming:

- `map`: global world frame
- `odom`: local continuous frame (can drift)
- `base_link`: robot body
- sensor frames: `camera_link`, `lidar_link`, etc.

Only publish frames your node owns.

## Rosbag and Plotting

Record and replay:

```bash
rosbag record /chatter /tf
rosbag play <file.bag>
```

Plot topic fields:

```bash
rosrun rqt_plot rqt_plot
```

Inspect message fields before plotting:

```bash
rostopic type /<topic> | xargs rosmsg show
```

## RViz Example

```bash
rosrun rviz rviz
```

Or load a saved config:

```bash
rosrun rviz rviz -d /path/to/config.rviz
```

## Common Message Types

| Type                  | Python import                         | Typical use                  |
| --------------------- | ------------------------------------- | ---------------------------- |
| `std_msgs/String`     | `from std_msgs.msg import String`     | Text and lightweight status  |
| `geometry_msgs/Twist` | `from geometry_msgs.msg import Twist` | Velocity commands            |
| `sensor_msgs/Image`   | `from sensor_msgs.msg import Image`   | Camera streams               |
| `nav_msgs/Odometry`   | `from nav_msgs.msg import Odometry`   | Pose and velocity estimation |

## Build, Clean, Debug

```bash
# clean and rebuild
cd ~/catkin_ws
catkin_make clean
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

# logs
roscd log
less latest/rosout.log
```

Logging in Python:

```python
rospy.logdebug("debug")
rospy.loginfo("info")
rospy.logwarn("warn")
rospy.logerr("error")
```

## Service vs Action vs State Machine

| Tool    | Best for                     | Feedback           | Cancel/preempt         | Typical pattern                    |
| ------- | ---------------------------- | ------------------ | ---------------------- | ---------------------------------- |
| Service | Quick request/response       | No                 | No                     | Config reads, one-shot computation |
| Action  | Long task execution          | Yes                | Yes                    | Navigation, manipulation jobs      |
| SMACH   | Orchestration and logic flow | Via states/actions | Depends on state logic | Mission-level behavior             |
