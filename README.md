# `p_rosbridge_server_cpp`

The node is a C++ implementation of the [Rosbridge protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md).
It is close to a drop-in replacement of the Python `rosbridge_server`.
Only websocket connections are supported.

We use the Qt library for the Websocket implementation, included in the `libqt5websockets5-dev` ubuntu package.

The following compressions are supported :

- **JSON**
- **CBOR**
- **CBOR-RAW**

### Features of the RobotWebTools' rosbridge that are not implemented

- PNG compression
- fragmented packets
- authentification (`rosauth`)
- service server
- TCP and UDP transport (only Websocket)
- BSON mode
- Regex used to authorize only some specific topics or services

### Changes compared to the RobotWebTools' rosbridge

In addition the previous listed unimplemented features, we have some design changes :

- When using the `call_service` request, the `type` parameter is mandatory. RobotWebTools' rosbridge uses the rosmaster API to find the correct type.
- A watchdog is activated by default, see more details below
- CBOR requests are supported (service call, subscribe, advertise...). The response will be in CBOR only if `"compression":"cbor"` is present.

### Node watchdog

The watchdog is used to protect this node from freezes. If the main thead blocks, the process will terminate with a SIGABRT. If the `respawn="true"` attribute has been set in the `.launch` file, roslaunch will automatically the process.

### Qt version
Qt 5.14+ fixes some bugs in QWebsocketServer : [5s freeze on client disconnection during frame reception](https://github.com/qt/qtwebsockets/commit/b14f5f59a3ae96949e6a33302385a751d6448182) and [connection header size too big](https://github.com/qt/qtwebsockets/commit/cea5603ee1a56bb5d177f35ed3f884345875099e). Ubuntu 18.04 and 20.04 is missing these fixes.

If you want to apply these fixes you need to install the newer Qt in another directory (for example /opt/Qt5.15) and set the CMake variables ROSBRIDGE_FORCE_CUSTOM_QT and ROSBRIDGE_FORCE_CUSTOM_QT_DIR.

## Implementation details
### Binary data in JSON objects

`int8[]` and `uint8[]` arrays are encoding in base64. This is done in the same way as RobotWebTools' rosbridge.

For example, the following `sensors_msgs/CompressedImage` message

```python
m.header...
m.format = "jpeg"
m.data = [0,1,2,3,4,5,6,7,8,9]
```

Will be encoded as :

```json
{
    "header": {"seq":123,"stamp":{"secs":123,"nsecs":456},"frame_id":"frame_id"},"format":"jpeg",
    "data":"AAECAwQFBgcICQ=="
}
```

## Dependancies

- `libqt5websockets5-dev`
- `roscpp`
- `ros_babel_fish`
- `rosbridge_cpp_msgs`
- `std_msgs`

## Optionnal ROS parameters

- `port` (*int*) : Websocket TCP port. `0` will let the OS attribute a random port (default value: `9090`)
- `service_timeout` (*double*) : Timeout, in seconds, for service requests (default value: `5.0`)
- `pong_timeout_s` (*double*) : Timeout, in seconds, of the websocket connection for each client (default value: `30.0`)
- `watchdog_enabled` (*bool*) : Enable the watchdog (default value: `true`)
- `watchdog_timeout` (*double*) : Watchdog timeout in seonds (default value: `5.0`)

## Parameters set by the node

The parameter `actual_port` will be set on startup. This is useful if `port` configured to `0`.

## Published topics

- `client_count` (*std_msgs/Int32*) : Number of clients connected to the server.
- `connected_clients` (rosbridge_msgs/ConnectedClients**) : Information on all the connected clients.

## How to install in ROS noetic
```
sudo apt install python3-catkin-tools # optionnal
sudo apt install qtbase5-dev libqt5websockets5-dev ros-noetic-diagnostic-updater

source /opt/ros/noetic/setup.bash
mkdir -p ws/src
cd ws/src
git clone https://github.com/ixblue/rosbridge_server_cpp.git
git clone https://github.com/1r0b1n0/librosqt.git
git clone https://github.com/StefanFabian/ros_babel_fish
catkin build
```

## Tests

rosunit 1.15.8 is needed to make the unit tests work. When using ROS `melodic` the rosunit package must be updated manually.
