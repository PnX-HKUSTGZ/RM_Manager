# remote_adapter

This node subscribes to a `rm_message::msg::RemoteControl` topic and forwards selected channels to individual `std_msgs/msg/Float32` topics after normalization. It also publishes chassis/arm enable topics similar to the `remote_controller` node.

Parameters
- `remote_controller_topic` (string, default `/rm_manager/remote_control`): input topic
- `map.count` (int, default 3): number of mappings
- For each mapping index i (0..map.count-1) the following parameters are used:
  - `map.i.channel` (int, default i): which channel index to read (0..n)
  - `map.i.topic` (string, default `/remote_adapter/chan{i}`): output topic to publish Float32
  - `map.i.max` (double, default 0.5): max scale used in normalization (see below)
  - `map.i.invert` (bool, default false): if true, invert sign after normalization
- `chasis_enable_topic` (string, default `/enable_chasis`): (optional) topic to publish Bool for chassis enable
- `arm_enable_topic` (string, default `/enable_arm`): (optional) topic to publish Bool for arm enable

Normalization
The node uses the same normalization as `remote_controller`:

data = (chan_value - 1024) / 660 * max

Usage
1. Build the workspace (from workspace root):

```bash
colcon build --packages-select remote_adapter
```

2. Run the node (example):

```bash
ros2 run remote_adapter remote_adapter_node
```

3. To change mappings at launch, pass parameters via a launch file or `ros2 run` with `--ros-args -p` flags, or use a YAML params file.
