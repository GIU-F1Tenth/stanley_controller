# High-Speed Stanley Controller for F1Tenth

A ROS2 Python package implementing an advanced Stanley controller optimized for high-speed autonomous racing in F1Tenth competitions.

## Overview

This implementation features a high-performance Stanley controller with advanced algorithms specifically designed for F1Tenth racing:

-   **Look-ahead Path Tracking**: Speed-dependent lookahead distance for stability at high speeds
-   **Path-based Cross Track Error**: Accurate perpendicular distance calculation to path segments
-   **Adaptive Gain Control**: Speed and curvature-dependent gains for optimal performance
-   **Front Axle Reference**: More accurate control using front axle position
-   **CSV Path Loading**: Direct loading of racing lines with velocity profiles
-   **Speed Smoothing**: Prevents jerky acceleration changes
-   **Oscillation Damping**: Reduces high-frequency oscillations for stable control

## Key Features

### Advanced Control Algorithms

-   **Adaptive Stanley Controller**: Dynamically adjusts gains based on speed and path curvature
-   **Look-ahead Waypoint Selection**: Uses optimal lookahead distance for high-speed stability
-   **Path-based Cross Track Error**: Calculates perpendicular distance to path segments
-   **Velocity Profile Following**: Follows speed profiles from CSV waypoints
-   **Speed-dependent Lookahead**: Increases lookahead distance with speed

### High-Speed Optimizations

-   **Front Axle Control**: Uses front axle position for more responsive control
-   **Oscillation Damping**: Damping term reduces high-frequency steering oscillations
-   **Speed Smoothing**: Prevents abrupt speed changes that could destabilize the vehicle
-   **Curvature Adaptation**: Adjusts gains based on path curvature

### CSV Path Input

-   **x, y, v Format**: Loads waypoints with position and velocity data
-   **Header Support**: Automatically detects and skips header rows
-   **Flexible Format**: Robust parsing handles various CSV formatsller

A ROS2 Python package implementing the Stanley controller for path tracking in F1Tenth autonomous racing.

## Overview

## Overview

This implementation features a high-performance Stanley controller with advanced algorithms specifically designed for F1Tenth racing. The Stanley controller is a path tracking algorithm that computes steering commands to follow a given reference path. It combines cross-track error correction with heading error correction to provide smooth and stable path following.

## Features

-   Stanley controller implementation for path tracking
-   Configurable control parameters via YAML configuration
-   Speed adaptation based on steering angle (slower for sharp turns)
-   ROS2 standard package structure
-   Launch file for easy deployment

## Package Structure

```
stanley_controller/
├── config/
│   └── stanley_params.yaml      # Configuration parameters
├── launch/
│   └── stanley_controller_launch.py  # Launch file
├── stanley_controller/
│   ├── __init__.py
│   └── stanley_controller_node.py    # Main controller node
├── resource/
│   └── stanley_controller
├── package.xml                       # Package metadata
├── setup.py                         # Python package setup
├── setup.cfg                        # Setup configuration
└── README.md
```

## Dependencies

-   `rclpy` - ROS2 Python client library
-   `std_msgs` - Standard ROS2 messages
-   `geometry_msgs` - Geometry-related messages
-   `nav_msgs` - Navigation messages
-   `sensor_msgs` - Sensor messages
-   `ackermann_msgs` - Ackermann steering messages

## Topics

### Subscribed Topics

-   `/ego_racecar/odom` (`nav_msgs/Odometry`) - Vehicle odometry
-   `/reference_path` (`nav_msgs/Path`) - Reference path to follow

### Published Topics

-   `/drive` (`ackermann_msgs/AckermannDriveStamped`) - Drive commands
-   `/cross_track_error` (`std_msgs/Float64`) - Cross track error for debugging

## Parameters

The controller can be configured via the `config/stanley_params.yaml` file:

-   `k_p`: Proportional gain for cross track error (default: 0.5)
-   `k_d`: Gain for heading error (default: 0.3)
-   `max_speed`: Maximum speed in m/s (default: 2.0)
-   `min_speed`: Minimum speed in m/s (default: 0.5)
-   `wheelbase`: Vehicle wheelbase in meters (default: 0.3302)
-   `max_steering_angle`: Maximum steering angle in radians (default: 0.4189)

## Usage

### Prepare Your CSV Path File

Create a CSV file with your racing line waypoints:

```csv
x,y,v
0.0,0.0,2.0
1.0,0.1,3.5
2.0,0.5,4.0
3.0,1.2,5.0
4.0,2.1,6.0
```

### Configure the Controller

Edit `config/stanley_params.yaml` to set your CSV path:

```yaml
stanley_controller:
    ros__parameters:
        path_csv_file: "/path/to/your/racing_line.csv"
        max_speed: 8.0 # Adjust based on your track
        k_p: 0.8 # Tune for your vehicle
```

### Build and Run

```bash
# Build the package
cd /path/to/your/ros2_ws
colcon build --packages-select stanley_controller
source install/setup.bash

# Run with CSV path
ros2 launch stanley_controller stanley_controller_launch.py \
  config_file:=/path/to/your/config.yaml

# Or run directly
ros2 run stanley_controller stanley_controller_node \
  --ros-args -p path_csv_file:="/path/to/racing_line.csv"
```

## Algorithm

The Stanley controller uses the following control law:

```
δ = ψ + arctan(k * e / v)
```

Where:

-   `δ` = steering angle
-   `ψ` = heading error (angle between vehicle heading and path direction)
-   `k` = proportional gain
-   `e` = cross track error (distance from vehicle to path)
-   `v` = vehicle speed

## Tuning Guidelines

1. **k_p (Cross Track Gain)**: Higher values provide more aggressive correction of cross track error but may cause oscillations
2. **max_speed/min_speed**: Adjust based on your vehicle's capabilities and track conditions
3. **max_steering_angle**: Should match your vehicle's physical steering limits

## Contributing

Please follow ROS2 coding standards before submitting contributions.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
