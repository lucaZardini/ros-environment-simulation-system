# Custom ROS messages

This folder contains custom ROS message definitions used in the project.

## What are Custom ROS Messages?

In ROS, messages are the primary way that nodes communicate with each other. While ROS provides many standard message types, custom messages can be defined to meet the specific needs of a project. Custom messages are defined in `.msg` files and allow you to specify the data structure that will be used for communication between nodes.

A `.msg` file consists of field names and types. Once defined, custom messages need to be compiled so that ROS nodes can use them.

- `robot_data.msg`: Message used to communicate the robot's state, it holds the id of each robot's namespace, the height of the target seen from the camera and the state (x, y, theta) with the corresponding uncertainties
- `uwb_data.msg`: Message used to identify the UWB tag with tag_id. It communicates the distance between the tag and the robot with a float
- `target_data.msg`: Message used to communicate the target discover, it holds the id of the robot and the target's position (x, y, z)
- `target_assignment_data`: Message used to communicate the target assignment, it holds the id of the sender robot, the id of the assigned robot and the target's position (x, y, z)
- `targets_data.msg`: Message used to communicate all the targets found, it holds a list of `target_data` messages