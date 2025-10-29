# twist_servo_debugger

Subscribe to `rm_message::msg::RemoteControl`, map channels to Twist axes, and publish a `geometry_msgs/msg/TwistStamped` message.

Parameters
- `remote_controller_topic` (string, default `/rm_manager/remote_control`)
- `output_topic` (string, default `/twist_servo_debugger/twist_stamped`)
- `frame_id` (string, default `base_link`) — header frame_id for published TwistStamped
You can configure mappings either with top-level parameter names or under a `map:` namespace. Top-level form (preferred):

- `channels` (list[int]) — remote channels
- `axes` (list[string]) — which axis each channel maps to, e.g. `linear.x`, `angular.z`
- `max` (list[double]) — scale/maximum for normalization
- `invert` (list[bool]) — invert sign if true

Or the namespaced (backward-compatible) form:

- `map.channels`, `map.axes`, `map.max`, `map.invert`

Example params (see `params/example.yaml`)

Usage
1. Build package:

```bash
colcon build --packages-select twist_servo_debugger
```

2. Source and launch:

```bash
source install/setup.bash
ros2 launch twist_servo_debugger example.launch.py
```
