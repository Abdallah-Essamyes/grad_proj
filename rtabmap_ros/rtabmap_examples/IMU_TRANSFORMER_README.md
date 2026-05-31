# IMU Axis Transformer - Quick Start Guide

## What is this?

The IMU Axis Transformer converts IMU data from the OAK-D camera's optical frame to ROS REP-103 standard, enabling proper integration with RTAB-Map SLAM.

## Quick Test

```bash
# Terminal 1 - Run the transformer
cd /home/ggsya/ros2_ws
source install/setup.bash
ros2 run rtabmap_examples imu_axis_transformer.py

# Terminal 2 - Test it
source install/setup.bash
python3 src/rtabmap_ros/rtabmap_examples/test/test_imu_integration.py
```

## What does it do?

**Input:** IMU data on `/imu` topic (Optical frame: Right-Down-Forward)  
**Output:** IMU data on `/imu/transformed` topic (ROS REP-103: Forward-Left-Up)

## Transformation Example

| Component | Input (Optical) | Output (ROS REP-103) |
|-----------|----------------|---------------------|
| accel_x   | 1.0 (right)    | 3.0 (forward)       |
| accel_y   | 2.0 (down)     | -1.0 (left)         |
| accel_z   | 3.0 (forward)  | -2.0 (up)           |
| frame_id  | oak_imu_frame  | oak-d-base-frame    |

## Parameters

- `enable_imu_transform` (default: true)
  - `true`: Transform IMU data (normal operation)
  - `false`: Pass through unchanged (bypass mode)

## Usage in Launch Files

```python
from launch_ros.actions import Node

Node(
    package='rtabmap_examples',
    executable='imu_axis_transformer.py',
    name='imu_transformer',
    parameters=[{'enable_imu_transform': True}],
    remappings=[
        ('/imu', '/oak/imu'),  # Subscribe to OAK-D IMU
        ('/imu/transformed', '/imu/data')  # Publish as standard /imu/data
    ]
)
```

## Testing

**Unit Tests (7 tests):**
```bash
python3 src/rtabmap_ros/rtabmap_examples/test/test_imu_axis_transformer.py -v
```

**Integration Test:**
Requires transformer node running in separate terminal.

## Topics

- **Subscribe:** `/imu` (sensor_msgs/Imu)
- **Publish:** `/imu/transformed` (sensor_msgs/Imu)

## Frame IDs

- **Input:** Any frame (typically `oak_imu_frame`)
- **Output:** `oak-d-base-frame`

## Documentation

See [PHASE1_IMPLEMENTATION.md](PHASE1_IMPLEMENTATION.md) for complete technical details.

## Status

✅ **FULLY IMPLEMENTED AND TESTED**
- All 7 unit tests passing
- Build system configured
- Ready for integration with imu_filter_madgwick

