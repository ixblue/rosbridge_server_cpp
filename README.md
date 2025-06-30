# `p_rosbridge_server_cpp`

The node is a C++ implementation of the [Rosbridge protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md).
It is close to a drop-in replacement of the Python `rosbridge_server`.
Only websocket connections are supported.

We use the Qt library for the Websocket implementation, included in the `libqt5websockets5-dev` ubuntu package.

The following compressions are supported :

- **JSON**
- **CBOR**
- **CBOR-RAW**

### Authentication

The server will handle `auth` request messages as specified in the [Rosbridge protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros1/ROSBRIDGE_PROTOCOL.md#33-authentication-message). An instance of the `rosauth` service node must be running to actually process the authentication requests.

If the `require_auth` parameter is set to `true`, the server will not process any messages (including any subscription requests) until a successful authentication has been performed, and will close sockets that have not authenticated after a timeout of 15s.

### Features of the RobotWebTools' rosbridge that are not implemented

- PNG compression
- fragmented packets
- service server
- TCP and UDP transport (only Websocket)
- BSON mode
- Regex used to authorize only some specific topics or services

### Changes compared to the RobotWebTools' rosbridge

In addition the previous listed unimplemented features, we have some design changes :

- When using the `call_service` request, the `type` parameter is mandatory. RobotWebTools' rosbridge uses the rosmaster API to find the correct type.
- A watchdog is activated by default, see more details below

### Node watchdog

The watchdog is used to protect this node from freezes. If the main thead blocks, the process will terminate with a SIGABRT. If the `respawn="true"` attribute has been set in the `.launch` file, roslaunch will automatically the process.

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
- `librosqt` : https://github.com/1r0b1n0/librosqt
- `ros_babel_fish`
- `rosbridge_cpp_msgs`
- `std_msgs`

## Optional ROS parameters

- `port` (*int*) : Websocket TCP port. `0` will let the OS attribute a random port (default value: `9090`)
- `service_timeout` (*double*) : Timeout, in seconds, for service requests (default value: `5.0`)
- `pong_timeout_s` (*double*) : Timeout, in seconds, of the websocket connection for each client (default value: `30.0`)
- `watchdog_enabled` (*bool*) : Enable the watchdog (default value: `true`)
- `watchdog_timeout` (*double*) : Watchdog timeout in seonds (default value: `5.0`)
- `require_auth` (*bool*) : Require authentication (default value: `false`)
- `require_ssl` (*bool*) : Start up the Websocket Server in secure mode (requires `ssl_cert_file` and `ssl_key_file` to be given) (default value: `false`)
- `ssl_cert_file` (*string*) : Path to the SSL certificate file (default value: `""`)
- `ssl_key_file` (*string*) : Path to the SSL key file (default value: `""`)

## Parameters set by the node

The parameter `actual_port` will be set on startup. This is useful if `port` configured to `0`.

## Published topics

- `client_count` (*std_msgs/Int32*) : Number of clients connected to the server.
- `connected_clients` (rosbridge_msgs/ConnectedClients**) : Information on all the connected clients.

## How to install in ROS noetic
```
sudo apt install python3-catkin-tools # optionnal
sudo apt install qtbase5-dev libqt5websockets5-dev ros-noetic-ros-babel-fish ros-noetic-diagnostic-updater

source /opt/ros/noetic/setup.bash
mkdir -p ws/src
cd ws/src
git clone https://github.com/ixblue/rosbridge_server_cpp.git
git clone https://github.com/1r0b1n0/librosqt.git
catkin build
```

## Tests

rosunit 1.15.8 is needed to make the unit tests work. When using ROS `melodic` the rosunit package must be updated manually.
