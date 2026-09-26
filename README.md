# leptrino_force_torque_ros_driver

This ROS 1 Noetic driver reads measurements from [Leptrino](https://www.leptrino.co.jp/)
six-axis force-torque sensors and publishes them as `geometry_msgs/WrenchStamped` messages.

The output topic is `~wrench`, which resolves to `/leptrino/wrench` with the default
node name, `leptrino`.

Tested with a PFS055YA251U6 sensor using communication protocol v1.31.
Protocol v1.13 is also supported, but changing the digital filter setting is not implemented.

## Launch

List the available serial devices to find your sensor's device ID:

```sh
ls -l /dev/serial/by-id/
```

Replace `<device-id>` with the corresponding entry name and launch the node:

```sh
roslaunch leptrino_force_torque_ros_driver leptrino.launch serial_port:="/dev/serial/by-id/<device-id>" pub_rate:=500
```

| Parameter / argument | Default | Description |
|---|---|---|
| `serial_port` | Required | Serial device path, such as `/dev/serial/by-id/<device-id>`. |
| `frame_id` | `leptrino` | Frame ID assigned to each `WrenchStamped` message. |
| `pub_rate` | `0.0` | Target average publication rate in Hz. `0` publishes the latest valid measurement from each receive chunk. |
| `pub_queue_size` | `1` | ROS publisher queue size. |
| `command_timeout` | `1.0` | Timeout in seconds for each command attempt, including transmission and response validation. |
| `command_retries` | `2` | Number of retries after the initial command attempt. |

## Design

### Processing flow

The driver retrieves product information, retrieves the sensor rated limits, and then requests
continuous transmission. Received measurements are converted and published in ROS as
`geometry_msgs/WrenchStamped` messages.
Handshake-based data acquisition and support for EXM0003 converter are not implemented.

### Protocol validation and conversion factors

Commands validate framing, escaping, BCC, lengths, sensor ID, command, and result codes,
with monotonic timeouts and bounded retries.
Initialization confirms STOP before querying configuration, preventing residual measurements
from being accepted as rated limits.
Limits are decoded explicitly as little-endian IEEE-754 values.
All six limits and their conversion factors (`rated_limit / 10000`) must be finite and positive
before any factors are committed; failed initialization prevents publication.

### Chunk reads and incremental parsing

The official sample program reads individual bytes and copies frames through a ring buffer.
This driver uses `poll()` and reads up to 4096 bytes at once, reducing system calls.
A bounded incremental parser handles split or multiple frames and dispatches them directly
without an intermediate frame queue.
The published timestamp is the host reception time of the chunk that completes the measurement.

### Publication rate and processing load

Each receive chunk contributes at most one publication: its last complete, valid measurement.
Earlier measurements in that chunk are discarded, while partial frames continue into the next read.
`pub_rate:=0` publishes each chunk's latest measurement; positive values additionally select chunks
at a target average rate using a monotonic schedule, without catch-up bursts after delays.
Reception, validation, and status checks continue at full rate, including discarded measurements.
Only selected measurements are converted and used to construct ROS messages, reducing downstream load.
Chunks without a new valid measurement never republish a previous value.
The same loop handles reception and publication, without a publication timer or shared-value mutex.
The achieved rate depends on chunk arrivals; selection does not perform averaging or anti-alias filtering.
