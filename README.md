# ros2-gst-test package

The goal of this package is to solve the following challenge:

Write code for two ROS2 Nodes, let's call them client and server:

- The nodes should each implement a GStreamer Pipeline for sending and receiving an UDP Stream
- The server node should stream a UDP Stream to the client side
- You don’t have to use two PC’S, you could just stream over localhost
- On start the pipelines should be in paused or ready state.
- On given user input (button press for example) from the client side should toggle the stream on the server side from paused to playing
- For the stream use a Videotestsrc with 600 x 400 @30 fps
- On another user input from the client the stream should be symmetrically cropped to 400 x 400

## Instructions

```bash
# Create workspace
cd ~/
mkdir -p gst_ws/src/ && cd gst_ws

# Create package, compile to validate configuration
ros2 pkg create ros2-gst-test --build-type ament_cmake --dependencies rclcpp std_msgs
colcon build
```
