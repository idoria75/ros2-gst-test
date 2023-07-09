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

## Implementation plan

- Implement prototype of service server and client using std_srvs SetBool
- Implement logic to switch between 4 states: PAUSED or PLAYING, 600x400 or 400x400
- Integrate GStreamer elements states PAUSED and playing fixed at 600x400
- Integrate GStreamer elements to change between 600x400 and 400x400

Two different services will be deployed because it may be useful to change only the resolution without changing the state. This also allows the server to deal with all the logic with regards to request handling. The client just needs to know the target state and receive confirmation. Also, multiple clients could be connected and making requests.
