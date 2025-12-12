# Custom Interfaces Package

This package contains custom message and service definitions for use in ROS 2.

## Structure
- `msg/` - Custom message definitions
- `srv/` - Custom service definitions
- `package.xml` - Package manifest with dependencies
- `CMakeLists.txt` - Build configuration

## Custom Message Definitions

### SensorData.msg
```
float64 temperature
float32 humidity
int32[] readings
string sensor_name
bool is_active
```

## Custom Service Definitions

### CalculateSum.srv
```
# Request
float64 first_number
float64 second_number
---
# Response
float64 sum
string error_message
```

## Building the Package

To build this package, create a workspace and use colcon:

```bash
# Create workspace
mkdir -p ~/ros2_custom_interfaces_ws/src
cd ~/ros2_custom_interfaces_ws/src

# Copy this package to the src directory
# (Assuming you've cloned or copied this package)

# Build the package
cd ~/ros2_custom_interfaces_ws
colcon build --packages-select custom_interfaces_pkg

# Source the workspace
source install/setup.bash

# Verify interfaces are available
ros2 interface show custom_interfaces_pkg/msg/SensorData
ros2 interface show custom_interfaces_pkg/srv/CalculateSum
```

## Using the Custom Interfaces

After building, you can use these interfaces in your Python or C++ code:

### Python Usage
```python
from custom_interfaces_pkg.msg import SensorData
from custom_interfaces_pkg.srv import CalculateSum
```

### C++ Usage
```cpp
#include "custom_interfaces_pkg/msg/sensor_data.hpp"
#include "custom_interfaces_pkg/srv/calculate_sum.hpp"
```

## Dependencies

This package requires:
- `rosidl_default_generators` (for interface generation)
- `std_msgs`, `geometry_msgs`, `builtin_interfaces` (for standard message types)
- `rclcpp`, `rclpy` (for ROS client libraries)