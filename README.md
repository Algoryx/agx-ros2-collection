## AGX ROS2 collection

Collection of ROS2 packages that with libraries, tutorials and resources for using ROS2 together with AGX Dynamics.
Clone this repository into your workspace/src directory and build running the ``colcon build`` command from your workspace directory.
Created by Algoryx Simulation AB under the Apache-2.0 license.

### Sending and receiving custom data types

By using the ``AnyMessageBuilder`` and ``AnyMessageParser`` it is possible to serialize and deserialize custom data types at runtime and send them as an ``agx_msgs::Any`` message.
To see an example of this, first build the ``agx_msgs``, ``agx_any_builder_parser`` and ``agx_any_msg_example`` packages.
Source the workspace you built these packages in and then run the sender example:

    > ros2 run agx_any_msg_example sender

In another terminal, source ROS2, your workspace and then run the receiver example:

    > ros2 run agx_any_msg_example receiver
