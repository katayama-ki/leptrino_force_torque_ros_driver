# leptrino_force_torque_ros_driver

This ROS 1 Noetic driver reads measurements from [Leptrino](https://www.leptrino.co.jp/)
six-axis force-torque sensors and publishes them as `geometry_msgs/WrenchStamped` messages.

The output topic is `~wrench`, which resolves to `/leptrino/wrench` with the default
node name, `leptrino`.

Tested with a PFS055YA251U6 sensor using communication protocol v1.31.
Protocol v1.13 is also supported, but changing the digital filter setting is not implemented.

## Usage

### Launch

List the available serial devices to find your sensor's device ID:

```sh
ls -l /dev/serial/by-id/
```

Replace `<device-id>` with the corresponding entry name and launch the node:

```sh
roslaunch leptrino_force_torque_ros_driver leptrino.launch serial_port:="/dev/serial/by-id/<device-id>"
```

The standard launch file exposes the commonly used private parameters below as launch arguments.

| Private parameter | Default | Description |
|---|---|---|
| `serial_port` | Required | Serial device path, such as `/dev/serial/by-id/<device-id>`. |
| `frame_id` | `leptrino` | Frame ID assigned to each `WrenchStamped` message. |
| `pub_rate` | `0.0` | Target average publication rate in Hz. `0` publishes the latest valid measurement from each receive chunk. |
| `pub_queue_size` | `1` | ROS publisher queue size. |
| `zero_wrench_on_start` | `false` | If true, zero the sensor on node startup and start publishing only after success. If startup zeroing fails, the node exits. |

#### Advanced parameters

The following parameters normally do not need adjustment.
To override them, set private parameters in your own launch file.

| Private parameter | Default | Description |
|---|---|---|
| `warning_timeout` | `1.0` | Time in seconds without a valid measurement frame before warning. |
| `command_timeout` | `1.0` | Timeout in seconds for each command attempt, including transmission and response validation. |
| `command_retries` | `2` | Number of retries after the initial command attempt. |
| `zero_wrench_samples` | `100` | Number of measurements to average for each zeroing request. |
| `zero_wrench_timeout` | `0.5` | Timeout in seconds for collecting all samples. |

### Zeroing

Call the private `~zero_wrench` service (`std_srvs/Trigger`):

```sh
rosservice call /leptrino/zero_wrench
```

A failed `zero_wrench` service call leaves the node running with the previous offset.

## Design and implementation

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
Only selected measurements are used to construct ROS messages, reducing downstream load.
During zeroing, healthy measurements are also converted for the sample average.
Chunks without a new valid measurement never republish a previous value.
The same loop handles reception and publication, without a publication timer.
A short mutex-protected section shares zeroing state with service callbacks; sample waiting
happens only on a callback thread, using a condition variable.
The achieved rate depends on chunk arrivals; selection does not perform averaging or anti-alias filtering.

### Sensor status and missing measurements

Repeated sensor-status and missing-measurement notifications are throttled to
once every five seconds per condition. The first missing-measurement warning
is issued after `warning_timeout`.

Overload (status bit 2) measurements remain publishable with a throttled WARN.
Values are not additionally clipped: the sensor saturates at raw counts of
±15000 for protocol v1.31 and ±32000 for v1.13, with rated load at ±10000.

Correction-data errors (bit 0) and sensor errors (bit 1) produce throttled ERROR
logs and suppress publication of the selected measurement. Reception continues,
and publication resumes when a healthy or overload-only measurement is selected.

After START is confirmed, `warning_timeout` monitors time since the last
protocol-valid measurement frame, independently of publication rate or zeroing.
Silence or only malformed frames produce a WARN after the threshold.
This warning neither stops nor restarts the sensor.

### Zeroing

Zeroing averages the next `zero_wrench_samples` valid measurements received after the request.
Each axis is summed and divided by the sample count once; the resulting offset is subtracted from subsequent wrench values. 

Service callbacks run sequentially on `ros::AsyncSpinner(1)`, separately from the hardware loop.
Requests received during zeroing wait in the callback queue. Each callback starts a fresh collection
and returns its own success or failure; queue waiting time is excluded from `zero_wrench_timeout`.

With `zero_wrench_on_start=true`, collection and its timeout start immediately after the START
response is confirmed, excluding initialization and START command wait time. No wrench is
published until zeroing succeeds. A timeout, including silence or too few healthy measurements,
logs a fatal error and exits the node. With startup zeroing enabled, the service is advertised only
after zeroing succeeds. Otherwise it is advertised when parsing of the receive chunk containing
the START response finishes, even if that chunk contains no measurements.
